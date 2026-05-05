import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rover_manager'),
        'config',
        'robot2_manager_params.yaml'
    )

    return LaunchDescription([
        # ----------------------------------------------------------------
        # Leo2 exploration waypoints
        # Adjust these (x, y, z) to match the area you want leo2 to cover.
        # Pick coordinates DIFFERENT from leo1's exp00..exp04 so the two
        # rovers explore distinct regions before the rock event.
        # ----------------------------------------------------------------
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-20", "30", "0", "0", "0", "0", "map", "exp10"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-15", "28", "0", "0", "0", "0", "map", "exp11"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-12", "25", "0", "0", "0", "0", "map", "exp12"]),

        # Post-rock waypoints (different from pre-rock)
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-18", "23", "0", "0", "0", "0", "map", "exp13"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-22", "27", "0", "0", "0", "0", "map", "exp14"]),

        # Leo2's home (its spawn point in 'map' frame)
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-17", "15", "30", "0", "0", "0", "map", "home2"]),

        # NOTE: do NOT republish 'rockg' here - it's already published by
        # leo1's manager.launch.py and TF will warn about double publishers.

        Node(
            package='rover_manager',
            executable='rover_manager',
            name='rover_manager',
            namespace='robot2',
            output='screen',
            parameters=[config],
        ),
    ])
