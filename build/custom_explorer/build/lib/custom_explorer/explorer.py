import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus as ActionGoalStatus
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose 
import math
from enum import Enum


class GoalStatus(Enum):
    """Status del goal corrente"""
    IDLE = 0
    SENDING = 1
    IN_PROGRESS = 2
    REACHED = 3
    UNREACHABLE = 4


class LunarExplorerNode(Node):
    def __init__(self):
        super().__init__('lunar_explorer')
        self.get_logger().info("🌙 Lunar Explorer Node Started (TF2 Mode)")

        self.callback_group = ReentrantCallbackGroup()

        # ==================== PARAMETERS ====================
        self.declare_parameter('robot_namespace', 'robot1')
        self.declare_parameter('exploration_area_size', 5.0)
        self.declare_parameter('grid_divisions', 3)
        self.declare_parameter('goal_retry_period', 10.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('waypoint_distance', 0.5)

        self.robot_ns = self.get_parameter('robot_namespace').value
        self.area_size = self.get_parameter('exploration_area_size').value
        self.grid_divisions = self.get_parameter('grid_divisions').value
        self.goal_retry_period = self.get_parameter('goal_retry_period').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.waypoint_distance = self.get_parameter('waypoint_distance').value

        # ==================== TF2 ====================
        # ✅ AGGIUNTO: TF Buffer e Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ==================== SUBSCRIBERS ====================
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_ns}/pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_ns}/odom',
            self.odom_callback,
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

        # ==================== STATE VARIABLES ====================
        self.robot_position = (0.0, 0.0)  # ✅ Sempre in frame /map
        self.robot_orientation = 0.0
        self.start_position = None
        self.pose_received = False
        self.pose_frame_id = None  # ✅ Traccia il frame della pose

        # Exploration zones
        self.zones = {}
        self.zones_initialized = False

        # Waypoint navigation
        self.current_zone = None
        self.zone_waypoints = []
        self.current_waypoint_idx = 0
        self.goal_status = GoalStatus.IDLE
        self.goal_handle = None
        self.goal_attempts = 0
        self.max_goal_attempts = 3

        # ======== PUNTI DI INTERESSE (in robot1/map) ========
        self.points_of_interest = [
            {'id': 1, 'position': (-7.0, 7.0), 'detected': False},
            {'id': 2, 'position': (0.0, 2.0), 'detected': False},
        ]

        self.detection_radius = 2.0  # raggio di visibilità della camera

        # Publisher dei POI rilevati
        self.poi_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_ns}/detected_points',
            10
        )

        # Timer per controllare continuamente se un POI è entrato nel range
        self.poi_timer = self.create_timer(
            0.5,
            self.check_points_of_interest,
            callback_group=self.callback_group
        )

        # ==================== TIMERS ====================
        self.goal_timer = self.create_timer(
            self.goal_retry_period,
            self.goal_check_callback,
            callback_group=self.callback_group
        )

        self.exploration_timer = self.create_timer(
            5.0,
            self.exploration_loop,
            callback_group=self.callback_group
        )

        self.get_logger().info(f"✅ Explorer ready: {self.robot_ns}")
        self.get_logger().info(f"📊 Area: {self.area_size}m, Waypoint: {self.waypoint_distance}m")

    # ==================== CALLBACKS ====================
    def pose_callback(self, msg):
        """✅ Callback pose - trasforma in frame /map"""
        if not self.pose_received:
            self.pose_received = True
            self.pose_frame_id = msg.header.frame_id
            self.get_logger().info(f"✅ Pose topic active (frame: {self.pose_frame_id})")

        # ✅ Trasforma pose in frame /map
        pose_in_map = self.transform_to_map(msg)
        
        if pose_in_map:
            self.robot_position = (
                pose_in_map.pose.position.x,
                pose_in_map.pose.position.y
            )

            if self.start_position is None:
                self.start_position = self.robot_position
                self.get_logger().info(
                    f"🎯 Start (in /map): ({self.start_position[0]:.2f}, {self.start_position[1]:.2f})"
                )

            q = pose_in_map.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Fallback su odom"""
        if self.pose_received:
            return

        # ✅ Odom è tipicamente già in frame corretto, ma verifica
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if self.start_position is None:
            self.start_position = self.robot_position
            self.get_logger().info(f"🎯 Start (odom): {self.start_position}")

    def transform_to_map(self, pose_stamped):
        """✅ Trasforma PoseStamped in frame /map usando TF2"""
        try:
            # Aspetta trasformazione (max 0.5 sec)
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                pose_stamped.header.frame_id,  # Source frame (es. robot1/map)
                rclpy.time.Time(),  # Latest available
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Applica trasformazione
            pose_in_map = do_transform_pose(pose_stamped, transform)
            return pose_in_map

        except Exception as e:
            self.get_logger().warn(
                f"⚠️ TF transform failed: {e}",
                throttle_duration_sec=5.0
            )
            return None

    # ==================== EXPLORATION LOGIC ====================
    def initialize_exploration_zones(self):
        """Crea griglia 3x3 CENTRATA sul robot (in frame /map)"""
        if self.start_position is None:
            return

        start_x, start_y = self.start_position
        zone_size = self.area_size / self.grid_divisions

        self.zones = {}
        zone_id = 0

        for i in range(self.grid_divisions):
            for j in range(self.grid_divisions):
                # ✅ Offset corretto
                grid_offset_i = i - (self.grid_divisions // 2)
                grid_offset_j = j - (self.grid_divisions // 2)
                
                # ✅ Centro zona in frame /map
                center_x = start_x + grid_offset_i * zone_size
                center_y = start_y + grid_offset_j * zone_size

                self.zones[zone_id] = {
                    'center': (center_x, center_y),
                    'explored': False,
                    'grid_pos': (i, j)
                }
                zone_id += 1

        self.zones_initialized = True
        self.get_logger().info(f"🗺️ Initialized {len(self.zones)} zones (in /map frame)")
        
        for zid, zdata in self.zones.items():
            cx, cy = zdata['center']
            dist = math.hypot(cx - start_x, cy - start_y)
            self.get_logger().info(f"  Zone {zid}: ({cx:.2f}, {cy:.2f}) - {dist:.2f}m")

    def get_nearest_unexplored_zone(self):
        """Trova zona non esplorata più vicina"""
        if not self.zones:
            return None

        nearest_zone = None
        min_distance = float('inf')

        for zone_id, zone_data in self.zones.items():
            if zone_data['explored']:
                continue

            center = zone_data['center']
            distance = math.hypot(
                self.robot_position[0] - center[0],
                self.robot_position[1] - center[1]
            )

            if distance < min_distance:
                min_distance = distance
                nearest_zone = zone_id

        return nearest_zone

    def generate_waypoints(self, target_x, target_y):
        """Genera waypoints intermedi"""
        waypoints = []
        
        start_x, start_y = self.robot_position
        
        dx = target_x - start_x
        dy = target_y - start_y
        total_distance = math.hypot(dx, dy)

        if total_distance <= self.waypoint_distance:
            waypoints.append((target_x, target_y))
            return waypoints

        num_waypoints = int(math.ceil(total_distance / self.waypoint_distance))
        
        for i in range(1, num_waypoints + 1):
            ratio = i / num_waypoints
            wp_x = start_x + dx * ratio
            wp_y = start_y + dy * ratio
            waypoints.append((wp_x, wp_y))

        return waypoints

    def exploration_loop(self):
        """Main exploration loop"""
        if not self.zones_initialized:
            if self.start_position is not None:
                self.initialize_exploration_zones()
            return

        if self.goal_status in [GoalStatus.SENDING, GoalStatus.IN_PROGRESS]:
            return

        if self.zone_waypoints and self.current_waypoint_idx < len(self.zone_waypoints):
            self.send_next_waypoint()
            return

        next_zone = self.get_nearest_unexplored_zone()

        if next_zone is None:
            self.get_logger().info("🎉 Exploration complete!")
            self.exploration_timer.cancel()
            self.goal_timer.cancel()
            return

        self.current_zone = next_zone
        zone_data = self.zones[next_zone]
        target_x, target_y = zone_data['center']
        
        self.zone_waypoints = self.generate_waypoints(target_x, target_y)
        self.current_waypoint_idx = 0
        
        self.get_logger().info(
            f"🎯 Zone {next_zone}: ({target_x:.2f}, {target_y:.2f}) → {len(self.zone_waypoints)} wp"
        )
        
        self.send_next_waypoint()

    def send_next_waypoint(self):
        """Invia prossimo waypoint"""
        if self.current_waypoint_idx >= len(self.zone_waypoints):
            self.zones[self.current_zone]['explored'] = True
            self.get_logger().info(f"✅ Zone {self.current_zone} EXPLORED")
            self.current_zone = None
            self.zone_waypoints = []
            self.goal_status = GoalStatus.IDLE
            return

        wp_x, wp_y = self.zone_waypoints[self.current_waypoint_idx]
        
        self.get_logger().info(
            f"📍 Waypoint {self.current_waypoint_idx + 1}/{len(self.zone_waypoints)}: "
            f"({wp_x:.2f}, {wp_y:.2f})"
        )
        
        self.send_navigation_goal(wp_x, wp_y)

    def goal_check_callback(self):
        """Check stato goal"""
        if self.goal_status != GoalStatus.IN_PROGRESS:
            return

        self.goal_attempts += 1
        
        if self.goal_attempts > self.max_goal_attempts:
            self.get_logger().warn(f"⚠️ Waypoint {self.current_waypoint_idx + 1} timeout, skip")
            self.skip_to_next_waypoint()

    def skip_to_next_waypoint(self):
        """Salta al prossimo waypoint"""
        if self.goal_handle:
            try:
                self.goal_handle.cancel_goal_async()
            except:
                pass

        self.current_waypoint_idx += 1
        self.goal_status = GoalStatus.IDLE
        self.goal_attempts = 0

    # ==================== NAVIGATION ====================
    def send_navigation_goal(self, x, y):
        """Invia goal a Nav2 (già in frame /map)"""
        self.goal_status = GoalStatus.SENDING
        self.goal_attempts = 0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'robot1/map'  # ✅ Frame corretto
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 not available")
            self.skip_to_next_waypoint()
            return

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback goal accepted/rejected"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Goal error: {e}")
            self.skip_to_next_waypoint()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rejected → skip")
            self.skip_to_next_waypoint()
            return

        self.goal_handle = goal_handle
        self.goal_status = GoalStatus.IN_PROGRESS
        self.get_logger().info("✅ Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """✅ Callback navigazione terminata"""
        try:
            result = future.result()
            
            if result.status == ActionGoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("🎯 Waypoint reached!")
                self.current_waypoint_idx += 1
                self.goal_status = GoalStatus.IDLE
                self.goal_attempts = 0
                
                # ✅ FIX: Processa subito prossimo waypoint o marca zona explored
                self.send_next_waypoint()
                
            else:
                self.get_logger().warn(f"⚠️ Nav failed ({result.status}) → skip")
                self.skip_to_next_waypoint()
                
        except Exception as e:
            self.get_logger().error(f"❌ Nav error: {e}")
            self.skip_to_next_waypoint()

    def check_points_of_interest(self):
        """Controlla se uno dei punti di interesse è nel campo visivo di 2m davanti al robot"""
        if not self.pose_received:
            return

        rx, ry = self.robot_position
        rtheta = self.robot_orientation

        for poi in self.points_of_interest:
            if poi['detected']:
                continue

            px, py = poi['position']
            # vettore robot → punto
            dx = px - rx
            dy = py - ry

            # distanza lungo direzione del robot (proiezione)
            distance_ahead = dx * math.cos(rtheta) + dy * math.sin(rtheta)
            # distanza perpendicolare
            lateral_offset = -dx * math.sin(rtheta) + dy * math.cos(rtheta)

            if 0 <= distance_ahead <= self.detection_radius and abs(lateral_offset) <= 0.5:
                # Punto rilevato
                self.publish_detected_point(poi)
                poi['detected'] = True
                self.get_logger().info(
                    f"🔎 Punto di interesse {poi['id']} rilevato a distanza {distance_ahead:.2f}m davanti!"
                )
    def publish_detected_point(self, poi):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.robot_ns}/map"  # frame del robot

        msg.pose.position.x = poi['position'][0]
        msg.pose.position.y = poi['position'][1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.poi_pub.publish(msg)
        self.get_logger().info(f"📡 Pubblicato punto {poi['id']}: {poi['position']}")


def main(args=None):
    rclpy.init(args=args)
    
    explorer = LunarExplorerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(explorer)

    try:
        explorer.get_logger().info("🚀 Exploration started")
        executor.spin()
    except KeyboardInterrupt:
        explorer.get_logger().info("🛑 Stopped")
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
