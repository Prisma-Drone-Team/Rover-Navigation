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
         #FIST MODIFY
        world_file_name = "moon.world" #"empty_world.sdf"
        #world_file_name = "empty_world.sdf"
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
                # 'robot1/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                # 'robot2/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/model/leo1/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/model/leo2/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/robot1/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/robot2/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/robot1/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/robot2/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/robot1/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/robot1/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/robot1/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/robot1/depth/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/robot1/depth/image_raw/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                '/robot2/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/robot2/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/robot2/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/robot2/depth/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/robot2/depth/image_raw/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            ],

            remappings=[

                ('/robot1/color/camera_info','/robot1/camera/color/camera_info'),
                ('/robot1/color/image_raw','/robot1/camera/color/image_raw'),
                ('/robot1/depth/image_raw/points','/robot1/camera/depth/color/points'),
                ('/robot1/depth/camera_info','/robot1/camera/aligned_depth_to_color/camera_info'),
                ('/robot1/depth/image_raw','/robot1/camera/aligned_depth_to_color/image_raw'),
                ('/robot2/color/camera_info','/robot2/camera/color/camera_info'),
                ('/robot2/color/image_raw','/robot2/camera/color/image_raw'),
                ('/robot2/depth/image_raw/points','/robot2/camera/depth/color/points'),
                ('/robot2/depth/camera_info','/robot2/camera/aligned_depth_to_color/camera_info'),
                ('/robot2/depth/image_raw','/robot2/camera/aligned_depth_to_color/image_raw'),

            ],

            parameters=[{
            # 'qos_overrides./color/camera_info.publisher.reliability': 'best_effort',
            # 'qos_overrides./color/camera_info.subscription.reliability': 'best_effort',
            'qos_overrides./robot1/camera/color/image_raw.publisher.reliability': 'best_effort',
            'qos_overrides./robot1/camera/color/image_raw.subscription.reliability': 'best_effort',
            'qos_overrides./robot1/camera/color/camera_info.publisher.reliability': 'best_effort',
            'qos_overrides./robot1/camera/color/camera_info.subscription.reliability': 'best_effort',
            'qos_overrides./robot1/camera/aligned_depth_to_color/image_raw.publisher.reliability': 'best_effort',
            'qos_overrides./robot1/camera/aligned_depth_to_color/image_raw.subscription.reliability': 'best_effort',
            'qos_overrides./robot1/camera/aligned_depth_to_color/camera_info.publisher.reliability': 'best_effort',
            'qos_overrides./robot1/camera/aligned_depth_to_color/camera_info.subscription.reliability': 'best_effort',
            'qos_overrides./robot1/camera/depth/color/points.publisher.reliability': 'best_effort',
            'qos_overrides./robot1/camera/depth/color/points.subscription.reliability': 'best_effort',
            'qos_overrides./robot2/camera/color/image_raw.publisher.reliability': 'best_effort',
            'qos_overrides./robot2/camera/color/image_raw.subscription.reliability': 'best_effort',
            'qos_overrides./robot2/camera/color/camera_info.publisher.reliability': 'best_effort',
            'qos_overrides./robot2/camera/color/camera_info.subscription.reliability': 'best_effort',
            'qos_overrides./robot2/camera/aligned_depth_to_color/image_raw.publisher.reliability': 'best_effort',
            'qos_overrides./robot2/camera/aligned_depth_to_color/image_raw.subscription.reliability': 'best_effort',
            'qos_overrides./robot2/camera/aligned_depth_to_color/camera_info.publisher.reliability': 'best_effort',
            'qos_overrides./robot2/camera/aligned_depth_to_color/camera_info.subscription.reliability': 'best_effort',
            'qos_overrides./robot2/camera/depth/color/points.publisher.reliability': 'best_effort',
            'qos_overrides./robot2/camera/depth/color/points.subscription.reliability': 'best_effort',
            # 'qos_overrides./imu/data.publisher.reliability': 'best_effort',
            # 'qos_overrides./imu/data.subscription.reliability': 'best_effort',
            # 'qos_overrides./livox/scan.publisher.reliability': 'best_effort',
            # 'qos_overrides./livox/scan.subscription.reliability': 'best_effort',
            # 'qos_overrides./livox/scan/points.publisher.reliability': 'best_effort',
            # 'qos_overrides./livox/scan/points.subscription.reliability': 'best_effort',
            }],
            output = 'screen',
        )

        return LaunchDescription([
            DeclareLaunchArgument('is_sim', default_value='true'),
            gazebo_ign,
            bridge,
            ])
