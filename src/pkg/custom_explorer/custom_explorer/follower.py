import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus as ActionGoalStatus
import math
from enum import Enum


class GoalStatus(Enum):
    """Status del goal corrente"""
    IDLE = 0
    SENDING = 1
    IN_PROGRESS = 2


class POIFollowerNode(Node):
    def __init__(self):
        super().__init__('poi_follower')
        self.get_logger().info("🚀 POI Follower Node Started")

        self.callback_group = ReentrantCallbackGroup()

        # ==================== PARAMETERS ====================
        self.declare_parameter('robot_namespace', 'robot2')
        self.robot_ns = self.get_parameter('robot_namespace').value

        self.waypoint_step = 1.0  # metri tra waypoint intermedi
        self.goal_retry_period = 5.0  # secondi per ritentare waypoint fallito

        # ==================== STATE ====================
        self.robot_position = (-5.0, 15.0)
        self.robot_orientation = 0.0
        self.pose_received = False
        self.goal_status = GoalStatus.IDLE
        self.goal_handle = None
        self.current_waypoint_idx = 0
        self.current_poi_waypoints = []
        self.poi_queue = []

        # ==================== SUBSCRIBERS ====================
        self.create_subscription(
            PoseStamped,
            '/robot1/detected_points',
            self.poi_callback,
            10,
            callback_group=self.callback_group
        )

        # ==================== ACTION CLIENT ====================
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            f'/{self.robot_ns}/navigate_to_pose',
            callback_group=self.callback_group
        )

        # ==================== TIMERS ====================
        self.create_timer(1.0, self.poi_navigation_loop)
        self.create_timer(self.goal_retry_period, self.retry_failed_goal)

    # ==================== CALLBACKS ====================
    def poi_callback(self, msg: PoseStamped):
        """Riceve un POI da robot1 e genera waypoint intermedi"""
        target_x = msg.pose.position.x + 1.0  # offset per evitare ostacolo
        target_y = msg.pose.position.y + 1.5

        waypoints = self.generate_waypoints_to_poi(target_x, target_y, step=self.waypoint_step)
        self.poi_queue.append(waypoints)
        self.get_logger().info(f"📥 POI ricevuto, {len(waypoints)} waypoint generati")

    def generate_waypoints_to_poi(self, target_x, target_y, step=1.0):
        """Genera waypoint intermedi ogni 'step' metri verso il target"""
        waypoints = []
        rx, ry = self.robot_position

        dx = target_x - rx
        dy = target_y - ry
        total_distance = math.hypot(dx, dy)

        if total_distance <= step:
            waypoints.append((target_x, target_y))
            return waypoints

        num_steps = int(math.ceil(total_distance / step))
        for i in range(1, num_steps + 1):
            ratio = i / num_steps
            wp_x = rx + dx * ratio
            wp_y = ry + dy * ratio
            waypoints.append((wp_x, wp_y))

        return waypoints

    def poi_navigation_loop(self):
        """Gestisce l'invio dei waypoint verso i POI in coda"""
        if self.goal_status in [GoalStatus.SENDING, GoalStatus.IN_PROGRESS]:
            return

        if not self.poi_queue and not self.current_poi_waypoints:
            return

        if not self.current_poi_waypoints:
            self.current_poi_waypoints = self.poi_queue.pop(0)
            self.current_waypoint_idx = 0

        if self.current_waypoint_idx < len(self.current_poi_waypoints):
            wp_x, wp_y = self.current_poi_waypoints[self.current_waypoint_idx]
            self.send_navigation_goal(wp_x, wp_y)

    # ==================== NAVIGATION ====================
    def send_navigation_goal(self, x, y):
        """Invia goal a Nav2 (frame /map) con orientamento verso il waypoint"""
        self.goal_status = GoalStatus.SENDING

        dx = x - self.robot_position[0]
        dy = y - self.robot_position[1]
        yaw = math.atan2(dy, dx)  # angolo verso il waypoint

        # DEBUG: stampo waypoint e angolo
        self.get_logger().info(f"🚀 Inviando waypoint a Nav2: ({x:.2f}, {y:.2f}), yaw: {math.degrees(yaw):.1f}°")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Converti yaw in quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 not available")
            return

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        """Callback goal accettato/rifiutato"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Goal error: {e}")
            self.goal_status = GoalStatus.IDLE
            return

        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rejected → ritento più tardi")
            self.goal_status = GoalStatus.IDLE
            return

        self.goal_handle = goal_handle
        self.goal_status = GoalStatus.IN_PROGRESS
        self.get_logger().info("✅ Goal accettato")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Callback navigazione terminata"""
        try:
            result = future.result()
            if result.status == ActionGoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"🎯 Waypoint {self.current_waypoint_idx + 1} raggiunto")
                self.robot_position = self.current_poi_waypoints[self.current_waypoint_idx]
                self.current_waypoint_idx += 1
                self.goal_status = GoalStatus.IDLE
                # Se ci sono altri waypoint verso lo stesso POI, continua
                if self.current_waypoint_idx >= len(self.current_poi_waypoints):
                    self.get_logger().info("🏁 POI raggiunto!")
                    self.current_poi_waypoints = []

            else:
                self.get_logger().warn(f"⚠️ Nav fallita ({result.status}) → ritento waypoint")
                self.goal_status = GoalStatus.IDLE  # ritenta al prossimo timer

        except Exception as e:
            self.get_logger().error(f"❌ Nav error: {e}")
            self.goal_status = GoalStatus.IDLE

    def retry_failed_goal(self):
        """Ritenta waypoint se fallito"""
        if self.goal_status == GoalStatus.IDLE and self.current_poi_waypoints:
            if self.current_waypoint_idx < len(self.current_poi_waypoints):
                wp_x, wp_y = self.current_poi_waypoints[self.current_waypoint_idx]
                self.get_logger().info(f"🔁 Ritento waypoint {self.current_waypoint_idx + 1}: ({wp_x:.2f}, {wp_y:.2f})")
                self.send_navigation_goal(wp_x, wp_y)


def main(args=None):
    rclpy.init(args=args)
    node = POIFollowerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
