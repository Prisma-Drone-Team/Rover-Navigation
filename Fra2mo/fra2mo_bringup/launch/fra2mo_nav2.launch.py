from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')

    params_file = os.path.join(
        get_package_share_directory('fra2mo_bringup'),
        'config',
        'copy_nav2_config.yaml'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='src/fra2mo_description/maps/mappa_prismalab.yaml',
        description='Absolute path to the map yaml file'
    )

    nav2_launch_include = GroupAction([
            SetRemap('/cmd_vel', '/fra2mo/cmd_vel'),
            SetRemap('/cmd_vel_smoothed', '/fra2mo/cmd_vel_smooth'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'params_file': params_file
                }.items()
            )
        ])

    return LaunchDescription([
        map_arg,
        nav2_launch_include
    ])
