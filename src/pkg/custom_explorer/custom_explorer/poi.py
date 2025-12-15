import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import math

class POIDetectorNode(Node):
    def __init__(self):
        super().__init__('poi_detector')
        self.get_logger().info("🌙 POI Detector Node Started")

        # ==================== PARAMETERS ====================
        self.declare_parameter('robot_namespace', 'robot1')
        self.robot_ns = self.get_parameter('robot_namespace').value

        # detection settings
        self.points_of_interest = [
            {'id': 1, 'position': (-7.0, 7.0), 'detected': False},
            {'id': 2, 'position': (0.0, 2.0), 'detected': False},
        ]
        self.detection_radius = 2.0  # metri

        # ==================== TF2 ====================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ==================== STATE ====================
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0
        self.pose_received = False
        self.offset_applied = False
        self.initial_offset = (0.0, 0.0)

        # ==================== SUBSCRIBERS ====================
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.robot_ns}/global_pose',
            self.pose_callback,
            10
        )
        self.create_subscription(
            Odometry,
            f'/{self.robot_ns}/odom',
            self.odom_callback,
            10
        )

        # ==================== PUBLISHER ====================
        self.poi_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_ns}/detected_points',
            10
        )

        # ==================== TIMER ====================
        self.create_timer(0.5, self.check_points_of_interest)
        self.create_timer(1.0, self.print_robot_position)

    # ==================== CALLBACKS ====================
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Aggiorna posizione dal global_pose (principal source)"""
        self.pose_received = True
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        pose_in_map = self.transform_to_map(pose_stamped)
        if pose_in_map:
            self.robot_position = (
                pose_in_map.pose.position.x,
                pose_in_map.pose.position.y
            )
            q = pose_in_map.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        """Fallback se global_pose non arriva"""
        if self.pose_received:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Applica offset iniziale solo la prima volta
        if not self.offset_applied:
            self.initial_offset = (-3.0, 4.0)  # offset iniziale robot rispetto a map
            self.offset_applied = True

        self.robot_position = (x + self.initial_offset[0], y + self.initial_offset[1])
        # Orientazione
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

    def transform_to_map(self, pose_stamped: PoseStamped):
        """Trasforma PoseStamped in frame /map usando TF2"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return do_transform_pose(pose_stamped, transform)
        except Exception:
            return pose_stamped  # se non riesce a trasformare, ritorna com’è

    # ==================== DETECTION ====================
    def check_points_of_interest(self):
        """Rileva POI entro un raggio attorno al robot"""
        rx, ry = self.robot_position

        for poi in self.points_of_interest:
            if poi['detected']:
                continue
            px, py = poi['position']
            distance = math.hypot(px - rx, py - ry)
            if distance <= self.detection_radius:
                poi['detected'] = True
                self.publish_detected_point(poi)
                self.get_logger().info(
                    f"🔎 Punto di interesse {poi['id']} rilevato a distanza {distance:.2f} m!"
                )

    def publish_detected_point(self, poi):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = poi['position'][0]
        msg.pose.position.y = poi['position'][1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.poi_pub.publish(msg)
        self.get_logger().info(f"📡 Pubblicato punto {poi['id']}: {poi['position']}")

    # ==================== DEBUG ====================
    def print_robot_position(self):
        x, y = self.robot_position
        # self.get_logger().info(f"🤖 Robot position: ({x:.2f}, {y:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = POIDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Detection stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
