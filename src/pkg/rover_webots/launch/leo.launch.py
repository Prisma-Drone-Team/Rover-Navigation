import os
import subprocess
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController

PRESETS = [
    {
        'x': '-40.0', 'y': '30.0', 'z': '13.0'
    },
    {
        'x': '-17.0', 'y': '15.0', 'z': '13.0'
    },
    {
        'x': '-17.0', 'y': '0.0', 'z': '13.0'
    },
    {
        'x': '-17.0', 'y': '-15.0', 'z': '13.0'
    },
    {
        'x': '-17.0', 'y': '-30.0', 'z': '13.0'
    }
]

# Conta il numero di robot già spawnati e decide dove spawnare il nuovo
def get_preset_index(name):
    try:
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
        nodes = result.stdout.split('\n')
        count = sum(1 for n in nodes if 'robot_state_publisher' in n)
        return count % len(PRESETS)
    except Exception:
        return 0

def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    
    preset_idx = get_preset_index(name)
    preset = PRESETS[preset_idx]
    
    urdf_file_name = "leo_webots.urdf"
    urdf_path = os.path.join(get_package_share_directory('rover_webots'), "urdf", urdf_file_name)
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read().replace('ROBOT_NAME', name)

    static_tf = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = f'map_to_{name}_map',
        arguments = [
            '--x', preset['x'],
            '--y', preset['y'],
            '--z', preset['z'],
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', f'{name}/map'
        ],
        parameters=[{'use_sim_time' : True}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name,            
        executable='robot_state_publisher',
        parameters=[{'robot_description' : robot_description, 'frame_prefix' : f'{name}/', 'use_sim_time' : True}],
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=name,
        parameters=[{'use_sim_time' : True}],
    )

    webots_spawn_z = str(float(preset['z']) + 3.0)
    spawn_rover = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/Ros2Supervisor/spawn_node_from_string',
            'webots_ros2_msgs/srv/SpawnNodeFromString',
            f'{{"data": "Rover {{ translation {preset["x"]} {preset["y"]} {webots_spawn_z} name \\"{name}\\" controller \\"<extern>\\" }}"}}'
        ],
        output='screen'
    )

    webots_driver = WebotsController(
        robot_name=name,
        namespace=name,
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        remappings=[
            (f'/{name}/{name}/d435_color/image_color', f'/{name}/color/image_raw'),
            (f'/{name}/{name}/d435_color/camera_info', f'/{name}/color/camera_info'),
            (f'/{name}/{name}/d435_depth/image', f'/{name}/depth/image_raw'),
            (f'/{name}/{name}/d435_depth/point_cloud', f'/{name}/depth/image_raw/points'),
            (f'/{name}/{name}/d435_depth/camera_info', f'/{name}/depth/camera_info'),
        ]
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_tf,
        spawn_rover,
        webots_driver,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('is_sim', default_value='true'),
        DeclareLaunchArgument('name', default_value='leo1'),
        OpaqueFunction(function=launch_setup)
    ])