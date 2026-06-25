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
        # Leo2's home (its spawn point in 'map' frame)
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-17", "15", "30", "0", "0", "0", "map", "home2"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["-11", "26", "0", "0", "0", "0", "map", "kick_point2"]),

        Node(
            package='rover_manager',
            executable='rover_manager',
            name='rover_manager',
            namespace='robot2',
            output='screen',
            parameters=[config],
        ),
    ])
