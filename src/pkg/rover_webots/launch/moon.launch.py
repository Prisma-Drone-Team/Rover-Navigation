import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher

def generate_launch_description():
    package_dir = get_package_share_directory('rover_webots')
    world_file = os.path.join(package_dir, 'worlds', 'moon.wbt')

    webots = WebotsLauncher(
        world=world_file,
        mode='realtime'
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    return LaunchDescription([
        DeclareLaunchArgument('is_sim', default_value='true'),
        webots,
        ros2_supervisor,
    ])
