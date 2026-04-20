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
        Node(
            package='rover_manager',
            executable='rover_manager',
            name='rover_manager',
            output='screen',
            parameters=[config],
        ),
    ])
