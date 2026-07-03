# nav_bringup.launch.py
#
# Versione UNIFICATA di:
#   namespace_1/2.launch.py + bringup_launch_1/2.py + navigation_launch_1/2.py
#
# Un solo file, parametrico su `namespace` e `params_file`: si lancia due volte,
# una per rover.
#
# Rispetto agli originali, si è rimosso perche' inutilizzato:
#   - ramo composition (use_composition / LoadComposableNodes / nav2_container)
#   - blocchi SLAM e localization (erano gia' commentati)
#   - argomenti: slam, map, map_server, default_bt_xml_filename
#
# Cosa e' stato mantenuto identico al setup attuale:
#   - /tf e /tf_static globali (un solo albero TF, frame distinti col frame_prefix)
#   - ReplaceString di <robot_namespace> nel file dei parametri
#   - RewrittenYaml con root_key = namespace
#   - PushRosNamespace(/robotN) su tutti i nodi

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    bringup_dir = get_package_share_directory('gz_migration')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # /tf e /tf_static restano GLOBALI (nomi assoluti): un solo albero TF condiviso,
    # i frame dei due rover si distinguono col frame_prefix (robotN/...).
    # Senza questo remapping, PushRosNamespace li sposterebbe su /robotN/tf e
    # romperebbe il setup. 
    remappings = [('/tf', '/tf'),
                  ('/tf_static', '/tf_static')]

    # Sostituisce il placeholder <robot_namespace> nel file dei parametri.
    # NOTA: replica esattamente il comportamento di bringup_launch_N: con
    # namespace='/robot1' 
    params_file_replaced = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)})

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file_replaced,
            root_key=namespace,
            param_rewrites={'use_sim_time': use_sim_time,
                            'autostart': autostart},
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # --- argomenti ---
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='/robot1',
        description='Namespace del rover, con slash iniziale (es. /robot1)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Usa il clock di Gazebo')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'leo1_navigation_params.yaml'),
        description='File dei parametri Nav2 per questo rover')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Avvio automatico dello stack Nav2')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Riavvia un nodo se va in crash')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='Livello di log')

    # --- nodi Nav2, tutti dentro il namespace del rover ---
    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings +
                    [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(bringup_group)
    return ld
