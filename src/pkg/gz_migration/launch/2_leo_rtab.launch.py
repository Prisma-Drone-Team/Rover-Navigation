import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
        DeclareLaunchArgument('is_sim', default_value='true'),
        # declare_x=DeclareLaunchArgument('spawn_x', default_value='5')
        # declare_y=DeclareLaunchArgument('spawn_y', default_value='0')
        # declare_z=DeclareLaunchArgument('spawn_z', default_value='0')
        # x = LaunchConfiguration('spawn_x')
        # y = LaunchConfiguration('spawn_y')
        # z = LaunchConfiguration('spawn_z')

        xacro_file_name = "leo2_rtab.urdf"
        xacro = os.path.join(get_package_share_directory('gz_migration'), "urdf", xacro_file_name)
        with open(xacro, 'r') as urdf_file:
            robot_description = urdf_file.read() #apri il file in modalità lettura e salvalo in robot_description (variabile) come stringa


        static_tf_1 = Node(
              package = 'tf2_ros',
              executable = 'static_transform_publisher',
              name = 'map_to_robot_map_2',
              arguments = ['-24.0', '10.0', '0.0', '0', '0', '0', 'map', 'robot2/map'], #-35 35 -2
        )
        
        static_tf_2 = Node(
              package = 'tf2_ros',
              executable = 'static_transform_publisher',
              name = 'robot_map_to_base_footprint',
              arguments = ['0', '0', '0', '0', '0', '0', 'robot1/map', 'robot1/base_footprint'],
        )

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            name='robot_state_publisher_2',
            executable='robot_state_publisher',
            parameters=[{'robot_description' : robot_description, 'frame_prefix' : 'robot2/'}],
            #parameters=[{'robot_description' : robot_description}],
            output='screen',
        )

        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_2',
            #remappings=[('joint_states', 'robot2/joint_states')],
        )

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name='urdf_spawner_2',
            output='screen',
            arguments=[
                '-name', 'leo2', 
                '-topic', 'robot_description', 
                '-x', '1.5',
                '-y', '8.0',
                '-z', '12',
                '-Y', '1.2',
                ],
            respawn=False,
        )

        odom_sim = Node(
            package='gz_migration',
            executable='robot2_odom_sim',
            name='odometry_simulation_2',
            parameters=[{'use_sim_time' : True}],
        )

        return LaunchDescription([
            robot_state_publisher_node,
            joint_state_publisher_node,
            spawn_node,
            # declare_x,
            # declare_y,
            # declare_z,
            odom_sim,
            static_tf_1,
            #static_tf_2,
            ])
