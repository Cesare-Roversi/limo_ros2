
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


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    params_file_path = os.path.join(
        get_package_share_directory('limo_bringup'),
        'param',
        'slam_toolbox_params.yaml'
    )
    
    

    # Launch the SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[params_file_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        slam_toolbox_node,
    ])
