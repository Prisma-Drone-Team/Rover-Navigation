import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('gz_migration')
    nav_launch_dir = os.path.join(package_dir, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'leo1_navigation_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    namespace = ['/robot1']

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time)

    bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'bringup_launch_1.py')),
                launch_arguments={  
                                'slam': 'False',
                                'namespace': namespace,
                                'use_namespace': 'True',
                                'map': '',
                                'map_server': 'False',
                                'params_file': params_file,
                                'default_bt_xml_filename': os.path.join(
                                    get_package_share_directory('nav2_bt_navigator'),
                                    'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                'autostart': 'true',
                                'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                )

    ld.add_action(bringup_cmd)
    

    return ld