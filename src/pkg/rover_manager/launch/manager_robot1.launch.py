import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rover_manager'),
        'config',
        'robot1_manager_params.yaml'
    )

    return LaunchDescription([
        
        # Leo1's home (spawn)
          Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-40", "30", "0", "0", "0", "0", "map", "home"]),
          Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "map", "robot1/map"]),
        # Shared landmarks (defined once here, used by both rovers)
          Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-26", "24", "0", "0", "0", "0", "map", "rockg"]),
        # Joint-mapping viewpoints around rockb (-21, 29)
          Node(package="tf2_ros", executable="static_transform_publisher",
               arguments=["-23.5", "29.0", "0", "0", "0", "0", "map", "view_b_w"]),
          Node(package="tf2_ros", executable="static_transform_publisher",
               arguments=["-21.0", "31.5", "0", "-1.5708", "0", "0", "map", "view_b_n"]),
          Node(package="tf2_ros", executable="static_transform_publisher",
               arguments=["-18.5", "29.0", "0", "3.1416", "0", "0", "map", "view_b_e"]),
          Node(package="tf2_ros", executable="static_transform_publisher",
               arguments=["-21.0", "26.5", "0", "1.5708", "0", "0", "map", "view_b_s"]),
          Node(package="tf2_ros", executable="static_transform_publisher",
               arguments=["-27", "25", "0", "0", "0", "-0.9", "map", "look_rock"]),
        # Rock inspection viewpoints (orbit around  the rockg)
          Node(package="tf2_ros", executable="static_transform_publisher",
               arguments=["-27.5", "24.0", "0", "0.0", "0", "0", "map", "view_w"]),
          Node(package="tf2_ros", executable="static_transform_publisher",
               arguments=["-25.0", "26.5", "0", "-1.5708", "0", "0", "map", "view_n"]),
          Node(package="tf2_ros", executable="static_transform_publisher",
               arguments=["-22.5", "24.0", "0", "3.1416", "0", "0", "map", "view_e"]),
          Node(package="tf2_ros", executable="static_transform_publisher",
               arguments=["-25.0", "21.5", "0", "1.5708", "0", "0", "map", "view_s"]),

          Node(
            package='rover_manager',
            executable='rover_manager',
            name='rover_manager',
            namespace='robot1',
            output='screen',
            parameters=[config],
          ),
    ])
