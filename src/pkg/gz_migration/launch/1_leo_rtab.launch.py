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

        xacro_file_name = "leo1_rtab.urdf"
        xacro = os.path.join(get_package_share_directory('gz_migration'), "urdf", xacro_file_name)
        with open(xacro, 'r') as urdf_file:
            robot_description = urdf_file.read() #apri il file in modalità lettura e salvalo in robot_description (variabile) come stringa


        # static_tf_1 = Node(
        #       package = 'tf2_ros',
        #       executable = 'static_transform_publisher',
        #       name = 'map_to_robot_map_1',
        #       arguments = ['-3.0', '4.0', '0', '0', '0', '0', 'map', 'robot1/map'], #-40 40 -1.5 #-20 10 15
        # )

        static_tf_1 = Node(
              package = 'tf2_ros',
              executable = 'static_transform_publisher',
              name = 'map_to_robot_map_1',
              arguments = ['-3.0', '4.0', '13', '0', '0', '0', 'map', 'robot1/map'],
              parameters=[{'use_sim_time' : True}],
        )
        
        static_tf_2 = Node(
              package = 'tf2_ros',
              executable = 'static_transform_publisher',
              name = 'robot_map_to_base_footprint_1',
              arguments = ['0', '0', '0', '0', '0', '0', 'robot1/map', 'robot1/base_footprint'],
        )

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            name='robot_state_publisher_1',
            namespace='robot1',            
            executable='robot_state_publisher',
            #parameters=[{'robot_description' : robot_description, 'frame_prefix' : 'robot1/'}],
            #parameters=[{'robot_description' : robot_description}],
            parameters=[{'robot_description' : robot_description, 'frame_prefix' : 'robot1/', 'use_sim_time' : True}],
            output='screen',
        )

        # joint_state_publisher_node = Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher_1',
        #     namespace='robot1',
        #     # remappings=[('joint_states', 'robot1/joint_states')],
        # )

        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_1',
            namespace='robot1',
            parameters=[{'use_sim_time' : True}],
        )

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name='urdf_spawner_1',
            output='screen',
            arguments=[
                '-name', 'leo1', 
                '-topic', 'robot1/robot_description', 
                # '-x', '-3.0',
                # '-y', '4.0',
                # '-z', '11',
                # '-x', '-10.0',
                # '-y', '15.0',
                # '-z', '13.0',
                '-x', '-40.0',
                '-y', '30.0',
                '-z', '13',
                '-Y', '0.0',
                ],
            respawn=False,
        )

        odom_sim = Node(
            package='gz_migration',
            executable='robot1_odom_sim',
            name='odometry_simulation_1',
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
