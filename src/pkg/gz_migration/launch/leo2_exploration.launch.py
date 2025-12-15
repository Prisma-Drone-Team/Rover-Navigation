from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_explorer',          # sostituisci con il nome del tuo package
            executable='follower',          # nome del nodo follower
            name='poi_follower_robot2',
            output='screen',
            parameters=[{
                'robot_namespace': 'robot2',    # namespace del secondo robot
                'poi_offset': 1.0               # offset di sicurezza rispetto ai POI
            }]
        )
    ])