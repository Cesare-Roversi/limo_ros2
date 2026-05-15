
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    share_limo_bringup = get_package_share_directory('limo_bringup')
    
    # ARGOMENTI:
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file_path = LaunchConfiguration('slam_params_file_path')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_slam_params_file_path = DeclareLaunchArgument(
        'slam_params_file_path',
        default_value=PathJoinSubstitution([
            share_limo_bringup, 'config', 'slam_toolbox_params.yaml'
        ])
    )
    #__ARGOMENTI
    
    

    # Launch the SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file_path, 
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file_path,
        slam_toolbox_node,
    ])
