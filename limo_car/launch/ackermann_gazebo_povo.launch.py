# Autor: Zhui Li
# E-Mail: lz554113510@gmail.com
# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.

import os
import shutil

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO

from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    # I check if gnome desktop environment + gnome terminal are being used:
    has_desktop_environment = os.environ.get('XDG_CURRENT_DESKTOP', 'NONE') != 'NONE'
    has_gnome_terminal = shutil.which('gnome-terminal') is not None


    # Define paths for model and world files
    package_name = 'limo_car'
    world_file_name = 'worlds/wall.sdf'
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_path = os.path.join(get_package_share_directory(package_name))

    description_pkg_path = os.path.join(get_package_share_directory('limo_description'))
    # world_path = os.path.join(description_pkg_path, world_file_name) #old
    # Wrap your path in expanduser
    world_path = os.path.expanduser('~/shared/3d_resources/world_povo/world_povo.sdf')

   
    # Position of the spawned entity
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.65' #! in teoria appena sopra il terreno
    spawn_yaw_val = '0.0'

    # Launch the ackermann launch
    mbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch', 'ackermann.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments={
			'gz_args': [
            world_path,
			' -r', 
			],
			'on_exit_shutdown': 'True',
			'paused': 'False',
			'use_sim_time': 'true'
		}.items(),
	)
    
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=[
                                   '-topic', 'robot_description',
                                   '-entity', 'mbot',
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val],
                        output='screen')
    
    robot_controllers = PathJoinSubstitution(
        [
            pkg_path,
            'config',
            'ackermann_drive_controller.yaml',
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--param-file',
                   robot_controllers,
                   ],
    )

    # wrap the existing spawner in a TimerAction to delay its execution
    delayed_joint_state_broadcaster_spawner = TimerAction(period=20.0, actions=[joint_state_broadcaster_spawner])
    

    # Bridge ROS topics and Gazebo messages for establishing communication
    # I check if I can start this node in a new gnome terminal:
    prefix_value=''
    if(has_desktop_environment and has_gnome_terminal):
        prefix_value='gnome-terminal --tab --'
        print("I will open parameter_bridge in a new terminal")
    else:
        print("I CANNOT open parameter_bridge in a new terminal")

    ros_gz_bridge = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		parameters=[{
			'config_file': os.path.join(pkg_path, 'config', 'ros_gz_bridge.yaml'),
		}],
		prefix=prefix_value, 
		output='screen'
	)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'config', 'limo_visual.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        mbot,
        gz_sim,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[delayed_joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_steering_controller_spawner],
            )
        ),
        TimerAction(
		period=10.0,  # delay in seconds
		actions=[ros_gz_bridge, rviz_node]
	)
        
    ])

