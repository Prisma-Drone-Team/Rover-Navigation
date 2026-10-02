import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

def matrix_to_quaternion(m):
    m00, m01, m02 = m[0], m[1], m[2]
    m10, m11, m12 = m[3], m[4], m[5]
    m20, m21, m22 = m[6], m[7], m[8]
    
    tr = m00 + m11 + m22
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    return [qx, qy, qz, qw]

class AbsoluteOdomPlugin:
    def init(self, webots_node, properties):
        try:
            self.__robot = webots_node.robot
            
            if not rclpy.ok():
                rclpy.init(args=None)
                
            self.__namespace = properties.get('namespace', '')
            self.__node = rclpy.create_node('diff_drive_plugin', namespace=self.__namespace)
            
            self.__left_motors = [
                self.__robot.getDevice('wheel_FL_joint'),
                self.__robot.getDevice('wheel_RL_joint')
            ]
            self.__right_motors = [
                self.__robot.getDevice('wheel_FR_joint'),
                self.__robot.getDevice('wheel_RR_joint')
            ]
            
            self.__left_sensors = [
                self.__robot.getDevice('wheel_FL_joint_sensor'),
                self.__robot.getDevice('wheel_RL_joint_sensor')
            ]
            self.__right_sensors = [
                self.__robot.getDevice('wheel_FR_joint_sensor'),
                self.__robot.getDevice('wheel_RR_joint_sensor')
            ]
            
            timestep = int(self.__robot.getBasicTimeStep())
            
            for m in self.__left_motors + self.__right_motors:
                if m is not None:
                    m.setPosition(float('inf'))
                    m.setVelocity(0.0)
                    m.setAcceleration(40.0)
                    
            for s in self.__left_sensors + self.__right_sensors:
                if s is not None:
                    s.enable(timestep)
                
            self.__target_twist = Twist()
            
            self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
            
            self.__odom_pub = self.__node.create_publisher(Odometry, 'odom', 10)
            self.__tf_broadcaster = TransformBroadcaster(self.__node)
            
            self.__x = 0.0
            self.__y = 0.0
            self.__z = 0.0
            self.__theta = 0.0
            self.__q = [0.0, 0.0, 0.0, 1.0]
            
            self.__prev_pos = [0.0, 0.0, 0.0]
            self.__start_pos = [0.0, 0.0, 0.0]
            self.__prev_time = self.__robot.getTime()
            self.__first_step = True
            
        except Exception as e:
            import traceback
            traceback.print_exc()

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        if rclpy.ok():
            rclpy.spin_once(self.__node, timeout_sec=0)
            
        L = 0.358
        R = 0.0625
        
        v = self.__target_twist.linear.x
        w = self.__target_twist.angular.z

        v_l = v - w * L / 2.0
        v_r = v + w * L / 2.0

        omega_l = v_l / R
        omega_r = v_r / R

        for m in self.__left_motors:
            if m is not None:
                m.setVelocity(omega_l)
        for m in self.__right_motors:
            if m is not None:
                m.setVelocity(omega_r)
                
        current_time = self.__robot.getTime()
        dt = current_time - self.__prev_time
        
        # Calcola l'odometria rispetto alle coordinate assolute del rover acquisite dal simulatore
        if dt > 0.0:
            robot_node = self.__robot.getSelf()
            if robot_node is not None:
                curr_pos = robot_node.getPosition()
                ori = robot_node.getOrientation()
                
                if self.__first_step:
                    self.__start_pos = curr_pos
                    self.__prev_pos = curr_pos
                    self.__prev_time = current_time
                    self.__first_step = False
                    return

                self.__x = curr_pos[0] - self.__start_pos[0]
                self.__y = curr_pos[1] - self.__start_pos[1]
                self.__z = curr_pos[2] - self.__start_pos[2]
                
                vx = (curr_pos[0] - self.__prev_pos[0]) / dt
                vy = (curr_pos[1] - self.__prev_pos[1]) / dt
                vz = (curr_pos[2] - self.__prev_pos[2]) / dt
                
                vtheta = w
                
                self.__prev_pos = curr_pos
                self.__prev_time = current_time
                
                self.__q = matrix_to_quaternion(ori)
            else:
                return

            sec = int(current_time)
            nanosec = int((current_time - sec) * 1e9)
            stamp = rclpy.time.Time(seconds=sec, nanoseconds=nanosec).to_msg()
            
            odom_frame = self.__namespace + '/odom' if self.__namespace else 'odom'
            base_frame = self.__namespace + '/base_footprint' if self.__namespace else 'base_footprint'
            
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = odom_frame
            msg.child_frame_id = base_frame
            msg.pose.pose.position.x = self.__x
            msg.pose.pose.position.y = self.__y
            msg.pose.pose.position.z = self.__z

            msg.pose.pose.orientation.x = self.__q[0]
            msg.pose.pose.orientation.y = self.__q[1]
            msg.pose.pose.orientation.z = self.__q[2]
            msg.pose.pose.orientation.w = self.__q[3]

            
            msg.twist.twist.linear.x = vx
            msg.twist.twist.angular.z = vtheta
            
            covariance = [
                0.05, 0.0,  0.0,  0.0, 0.0, 0.0,
                0.0,  0.05, 0.0,  0.0, 0.0, 0.0,
                0.0,  0.0,  0.01, 0.0, 0.0, 0.0,
                0.0,  0.0,  0.0,  0.1, 0.0, 0.0,
                0.0,  0.0,  0.0,  0.0, 0.1, 0.0,
                0.0,  0.0,  0.0,  0.0, 0.0, 0.2
            ]
            msg.pose.covariance = covariance
            msg.twist.covariance = covariance
            
            self.__odom_pub.publish(msg)
            
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = odom_frame
            t.child_frame_id = base_frame
            t.transform.translation.x = self.__x
            t.transform.translation.y = self.__y
            t.transform.translation.z = self.__z
            t.transform.rotation.x = self.__q[0]
            t.transform.rotation.y = self.__q[1]
            t.transform.rotation.z = self.__q[2]
            t.transform.rotation.w = self.__q[3]
            
            self.__tf_broadcaster.sendTransform(t)