import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

# Dati di configurazione standard (default: leo1)
HOME_POSE = '-40.0,30.0,13.0'
TFS = {
    'home': ['-40.0', '30.0', '13.0', '0.0', '0.0', '0.0'],
    'rockg': ['-25.0', '24.0', '15.26', '0.0', '0.0', '0.0'],
    'view_w': ['-27.5', '24.0', '15.26', '0.0', '0.0', '0.0'],
    'view_n': ['-25.0', '26.5', '15.26', '-1.5708', '0.0', '0.0'],
    'view_e': ['-22.5', '24.0', '15.26', '3.1416', '0.0', '0.0'],
    'view_s': ['-25.0', '21.5', '15.26', '1.5708', '0.0', '0.0'],
    'look_rock': ['-27.0', '25.0', '15.26', '0.0', '0.0', '-0.9']
}

def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    
    pkg_dir = get_package_share_directory('rover_manager')
    params_file = os.path.join(pkg_dir, 'config', 'manager_params.yaml')
    
    # Sostituisce dinamicamente <robot_name> con il nome effettivo
    configured_params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_name>': name})

    nodes = []
    
    # Nodo principale del Rover Manager
    manager_node = Node(
        package='rover_manager',
        executable='rover_manager',
        name='rover_manager',
        namespace=name,
        output='screen',
        parameters=[configured_params_file, {'return_to_base.home_pose': HOME_POSE}],
    )
    nodes.append(manager_node)
    
    # Genera i TF Publisher dinamici prefissati col nome del robot
    for tf_name, coords in TFS.items():
        tf_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"tf_{tf_name}_{name}",
            arguments=[
                '--x', coords[0],
                '--y', coords[1],
                '--z', coords[2],
                '--yaw', coords[3],
                '--pitch', coords[4],
                '--roll', coords[5],
                '--frame-id', 'map',
                '--child-frame-id', f"{name}/{tf_name}"
            ],
            parameters=[{'use_sim_time': True}]
        )
        nodes.append(tf_node)
        
    return nodes

def generate_launch_description():
    declare_name_cmd = DeclareLaunchArgument(
        'name',
        default_value='leo1',
        description='Name of the robot')

    ld = LaunchDescription()
    ld.add_action(declare_name_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld