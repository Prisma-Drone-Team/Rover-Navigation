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
        # Pre-rock exploration waypoints
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-32", "28", "0", "0", "0", "0", "map", "exp00"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-26", "28", "0", "0", "0", "0", "map", "exp01"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-26", "25", "0", "0", "0", "0", "map", "exp02"]),
        
        # Post-rock waypoints (different from pre-rock)
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-35", "20", "0", "0", "0", "0", "map", "exp03"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-38", "25", "0", "0", "0", "0", "map", "exp04"]),

        # Leo1's home (spawn)
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-40", "30", "0", "0", "0", "0", "map", "home"]),

        # Shared landmarks (defined once here, used by both rovers)
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-25", "24", "0", "0", "0", "0", "map", "rockg"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-27", "25", "0", "0", "0", "-0.9", "map", "look_rock"]),

        Node(
            package='rover_manager',
            executable='rover_manager',
            name='rover_manager',
            namespace='robot1',
            output='screen',
            parameters=[config],
        ),
    ])
