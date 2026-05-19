# Autor: Zhui Li
# E-Mail: lz554113510@gmail.com
# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.

import os

import launch

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():
    
    share_limo_car = os.path.join(get_package_share_directory('limo_car'))
    share_limo_description = os.path.join(get_package_share_directory('limo_description'))

    default_robot_xacro_file_path = os.path.join(share_limo_description, 'urdf', 'limo_ackermann_mycobot.xacro.urdf')

    #! di questa roba mi devo preoccupare???
    #Find the ros control plugin path
    ros_ctrl_plugin_dir = os.path.join(share_limo_car, 'src', 'gz_ros2_control')

    # Set the GZ_SIM_SYSTEM_PLUGIN_PATH environment variable
    plugin_paths = os.pathsep.join([
        '/opt/ros/humble/lib',
        '/usr/lib/',
        ros_ctrl_plugin_dir
    ])
    gz_plugin_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        plugin_paths
    )


    # PARAMETRI
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_xacro_file_path = LaunchConfiguration('robot_xacro_file_path')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true')
    declare_robot_xacro_file_path = DeclareLaunchArgument(
        'robot_xacro_file_path',
        default_value=default_robot_xacro_file_path,
        description='robot xacro file path'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', robot_xacro_file_path]), value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return LaunchDescription([
        gz_plugin_env,
        declare_use_sim_time,
        declare_robot_xacro_file_path,
        node_robot_state_publisher, 
        joint_state_publisher_node
    ])
