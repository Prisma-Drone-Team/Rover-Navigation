import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('gz_migration')
    nav_launch_dir = os.path.join(package_dir, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )
    ld.add_action(declare_use_sim_time)

    leo1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, '1_leo_rtab.launch.py')))
    
    leo2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, '2_leo_rtab.launch.py')))

    ld.add_action(leo1)
    ld.add_action(leo2)
    
    return ld