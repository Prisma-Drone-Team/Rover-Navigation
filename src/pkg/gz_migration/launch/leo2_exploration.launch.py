import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    exploration = Node(
        package = 'gz_migration',
        executable = 'robot2_exploration',
        name = 'exploration_node_2',
        parameters = [{'use_sim_time' : True}],
    )

    return LaunchDescription([
        exploration,
    ])