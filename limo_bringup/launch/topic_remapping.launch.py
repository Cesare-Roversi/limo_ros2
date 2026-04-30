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
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    

    # Relay per unire /tf_odometry in /tf
    tf_odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='tf_odom_relay',
        arguments=['/ackermann_steering_controller/tf_odometry', '/tf'], 
        parameters=[{'use_sim_time': True}], 
        output='screen'
    )



    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,     description='Use simulation (Gazebo) clock if true'),

        # Relay /tf_odometry -> /tf per avere l'albero TF completo (odom->base_link unito a base_link->resto)
        tf_odom_relay,

        GroupAction( 
            actions=[
                # Ricollega l'odometria: nav2 legge /odom, il controller pubblica sul suo topic
                SetRemap(src='/odom', dst='/ackermann_steering_controller/odometry'),
                
                # Ricollega i comandi di velocità
                SetRemap(src='/cmd_vel', dst='/ackermann_steering_controller/reference'),
                
    
            ]
        ),
    ])