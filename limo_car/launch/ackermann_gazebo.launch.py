# Autor: Zhui Li
# E-Mail: lz554113510@gmail.com
# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


from launch_ros.actions import Node

def generate_launch_description():

    # definiert Path für Modell
    package_name = 'limo_car'
    world_file_path = 'worlds/empty_world.model'
    rviz_path = 'rviz/gazebo.rviz'


    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, world_file_path)
    default_rviz_config_path = os.path.join(pkg_path, rviz_path)



    car_xacro_path = PathJoinSubstitution([pkg_path, 'gazebo', 'ackermann_with_sensor.xacro'])

    car_description_content = ParameterValue(Command(['xacro ', car_xacro_path]), value_type=str)

    

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Position dafür, wo die Modelle herstellt werden
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    mbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch', 'ackermann.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'world': world_path}.items()
    )

    # Einbindung der Gazebo-Startdatei, die im gazebo_ros-Paket enthalten ist
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    # )

    gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		# launch_arguments={
		# 	'gz_args': [PathJoinSubstitution([
		# 		drone_gazebo_dir,
		# 		'worlds',
		# 		gz_world_file, 
		# 	]),
		# 	' -r', 
		# 	'-s'
		# 	],
		# 	'on_exit_shutdown': 'True',
		# 	'paused': 'False',
		# 	'use_sim_time': 'true'
		# }.items(),
	)

    model_path = PathJoinSubstitution(
        [pkg_path, 'models', 'limo_car/urdf/limo_ackermann_base.xacro'])

    # spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     name='spawn_entity',
    #     output='screen',
    #     parameters=[{
    #         'world': 'default.sdf',  # Uncomment and set if needed
    #         # 'file': model_path,    # Uncomment and set if needed
    #         # 'model_string': model_path,
    #         # 'topic': 'robot_description',
    #         # 'entity_name': 'mbot',
    #         # 'allow_renaming': allow_renaming,  # Uncomment if needed
    #         # 'x': '0.0',
    #         # 'y': '0.0',
    #         # 'z': '0.0',
    #         # 'R': '0.0',
    #         # 'P': '0.0',
    #         # 'Y': '0.0',
    #     }]
    # )


    # spawn_entity = Node(
    # package='ros_gz_sim',
    # executable='create',
    # name='spawn_entity',
    # arguments=['-file', car_description_content.value, '-z', '0.0', '-name', 'limo_car'],
    # prefix='gnome-terminal --tab --',
    # output='screen'
    # )

    # laufen ein leere node aus den gazebo_ros package
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'mbot',
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val],
                        output='screen')


    return LaunchDescription([
        mbot,
        gz_sim,
        spawn_entity,
        rviz_arg,
        rviz_node
    ])