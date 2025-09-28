# Autor: Zhui Li
# E-Mail: lz554113510@gmail.com
# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable

import xacro


def generate_launch_description():

    # um die Zeit zu simulieren
    use_sim_time = LaunchConfiguration('use_sim_time')

    # finden path für Modell
    pkg_path = os.path.join(get_package_share_directory('limo_car'))
    description_path = os.path.join(get_package_share_directory('limo_description'))
    xacro_file = os.path.join(description_path, 'urdf', 'limo_ackermann.xacro.urdf')
    robot_description_config = xacro.process_file(xacro_file)

    # get the directory containing this launch file
    current_dir = os.path.dirname(os.path.realpath(__file__))

    # optional: log or print for debugging
    print(f"Current launch directory: {current_dir}")

    ros_ctrl_plugin_dir = os.path.join(pkg_path, 'src', 'gz_ros2_control')

    # Set the GZ_SIM_SYSTEM_PLUGIN_PATH environment variable
    plugin_paths = os.pathsep.join([
        '/opt/ros/humble/lib',
        ros_ctrl_plugin_dir
    ])
    gz_plugin_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        plugin_paths
    )
    # Erstellt ein robot_state_publisher Node
    params = {'robot_description': robot_description_config.toxml(),
            'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher
    ])
