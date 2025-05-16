from launch.substitutions import PathJoinSubstitution
import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

        world_file_name = "empty_world.sdf"
        world_file = os.path.join(get_package_share_directory('gz_migration'), "worlds", world_file_name)

        gazebo_ign = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
               [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
                launch_arguments=[('gz_args', ['-r -v4 ', world_file])]
        )

        bridge = Node(
            package = 'ros_gz_bridge',
            executable = 'parameter_bridge',
            arguments = [
                #'robot1/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/model/leo1/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                #'/world/default/pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V',
                #'/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/robot1/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                #'odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                #'/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                  #'/imu/data_raw@sensor_msgs/msg/Imu@gz.msgs.IMU',
                'robot1/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                'robot1/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                'robot1/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                'robot1/depth/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                #'robot1/depth/image_raw/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                'robot1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                'robot1/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            ],
            output = 'screen',
        )

        return LaunchDescription([
            DeclareLaunchArgument('is_sim', default_value='true'),
            gazebo_ign,
            bridge,
            ])