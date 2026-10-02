import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class DiffDrivePlugin:
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
                    m.setAcceleration(40.0) # Accelerazione graduale e non istantanea
                    
            for s in self.__left_sensors + self.__right_sensors:
                if s is not None:
                    s.enable(timestep)
                
            self.__target_twist = Twist()
            
            self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1) # Quando in cmd_vel arriva un messaggio di tipo Twist, viene chiamata __cmd_vel_callback
            
            self.__odom_pub = self.__node.create_publisher(Odometry, 'odom', 10) # Pubblica la posizione stimata nel topic odom
            self.__tf_broadcaster = TransformBroadcaster(self.__node)
            
            self.__x = 0.0
            self.__y = 0.0
            self.__theta = 0.0
            
            self.__prev_left_pos = 0.0
            self.__prev_right_pos = 0.0
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
            
        L = 0.358 # Distanza tra la ruota destra e sinistra
        R = 0.0625 # Raggio dela ruota
        
        v = self.__target_twist.linear.x
        w = self.__target_twist.angular.z

        # Se il robot va dritto la velocità sarà v
        v_l = v - w * L / 2.0
        v_r = v + w * L / 2.0

        # Weboots vuole rad/s
        omega_l = v_l / R
        omega_r = v_r / R

        # Si applica la velocità a tutte le ruote
        for m in self.__left_motors:
            if m is not None:
                m.setVelocity(omega_l)
        for m in self.__right_motors:
            if m is not None:
                m.setVelocity(omega_r)
                
        current_time = self.__robot.getTime()
        dt = current_time - self.__prev_time
        
        if dt > 0.0:
            # Calcolo quanto hanno ruotato le ruote e ne faccio la media per diminuire errori di slittamento di una sola ruota
            left_pos = 0.0
            left_count = 0
            for s in self.__left_sensors:
                if s is not None:
                    val = s.getValue()
                    if not math.isnan(val):
                        left_pos += val
                        left_count += 1
            if left_count > 0:
                left_pos /= left_count
            else:
                return
                
            right_pos = 0.0
            right_count = 0
            for s in self.__right_sensors:
                if s is not None:
                    val = s.getValue()
                    if not math.isnan(val):
                        right_pos += val
                        right_count += 1
            if right_count > 0:
                right_pos /= right_count
            else:
                return

            if self.__first_step:
                self.__prev_left_pos = left_pos
                self.__prev_right_pos = right_pos
                self.__prev_time = current_time
                self.__first_step = False
                return
                
            d_left = left_pos - self.__prev_left_pos
            d_right = right_pos - self.__prev_right_pos
            
            self.__prev_left_pos = left_pos
            self.__prev_right_pos = right_pos
            self.__prev_time = current_time

            # Ottengo la distanza percorsa dalle ruote
            dist_left = d_left * R
            dist_right = d_right * R
            
            d_x = (dist_right + dist_left) / 2.0 # Distanza centrale del robot percorsa
            d_theta = (dist_right - dist_left) / L # Rotazione del robot

            # Aggiorno la posizione assoluta del robot nella mappa
            self.__x += d_x * math.cos(self.__theta + d_theta / 2.0)
            self.__y += d_x * math.sin(self.__theta + d_theta / 2.0)
            self.__theta += d_theta

            # Creazione del messaggio            
            vx = d_x / dt
            vtheta = d_theta / dt

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
            msg.pose.pose.position.z = 0.0

            
            q = euler_to_quaternion(0, 0, self.__theta)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

            
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
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.__tf_broadcaster.sendTransform(t)