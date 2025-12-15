from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_explorer',
            executable='point_of_interest',
            name='lunar_explorer',
            output='screen',
            parameters=[{
                'robot_namespace': 'robot1',
                'exploration_area_size': 10.0,  # metri
                'grid_divisions': 3,  # 3x3 = 9 zone
                'goal_retry_period': 20.0,  # secondi
                'obstacle_threshold': 50,  # costmap value
                'goal_tolerance': 0.3,  # metri
                'waypoint_distance': 0.5, 
            }]
        )
    ])
