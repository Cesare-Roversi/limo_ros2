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

import xacro


def generate_launch_description():

    # Use the simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Find the model path
    pkg_path = os.path.join(get_package_share_directory('limo_car'))
    description_path = os.path.join(get_package_share_directory('limo_description'))
    xacro_file = os.path.join(description_path, 'urdf', 'limo_ackermann.xacro.urdf')
    robot_description_config = xacro.process_file(xacro_file)

    #Find the ros control plugin path
    ros_ctrl_plugin_dir = os.path.join(pkg_path, 'src', 'gz_ros2_control')

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

    # Do: export GZ_SIM_RESOURCE_PATH=/usr/share/gz/gz-sim8/:$GZ_SIM_RESOURCE_PATH

    # gz_resource_env = SetEnvironmentVariable(
    #     'GZ_SIM_RESOURCE_PATH',
    #     '/usr/share/gz/gz-sim8/:$GZ_SIM_RESOURCE_PATH'
    # )
    
    # Start a robot state publisher node
    params = {'robot_description': robot_description_config.toxml(),
            'use_sim_time': use_sim_time}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return LaunchDescription([
        gz_plugin_env,
        # gz_resource_env,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher, 
        joint_state_publisher_node
    ])
