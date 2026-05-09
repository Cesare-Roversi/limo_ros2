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
    
    # get share dirs
    limo_bringup_share_dir = get_package_share_directory('limo_bringup')
    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')
    limo_description_share_dir = get_package_share_directory('limo_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(limo_description_share_dir, 'maps/mappa_povo', 'povo.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(limo_bringup_share_dir, 'param', 'tesi_nav2_ackermann.yaml'))
    #! ESISTEVANO un file UGUALI ma con: nav2.yaml, navigation2.yaml
    
    rviz_config_path = LaunchConfiguration('rviz_config', default=os.path.join(nav2_bringup_share_dir, 'rviz', 'nav2_default_view.rviz'))

    # Relay per unire /tf_odometry in /tf
    tf_odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='tf_odom_relay',
        arguments=['/ackermann_steering_controller/tf_odometry', '/tf'], #! TENIAMO /tf (assumiamo namespace globale)
        parameters=[{'use_sim_time': True}], 
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,     description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map',          default_value=map_yaml_path,    description='Full path to map yaml file to load'),
        DeclareLaunchArgument('params_file',  default_value=nav2_param_path,  description='Full path to nav2 param file to load'),
        DeclareLaunchArgument('rviz_config',  default_value=rviz_config_path, description='Full path to rviz config file to load'),

        # Relay /tf_odometry -> /tf per avere l'albero TF completo (odom->base_link unito a base_link->resto)
        tf_odom_relay,

        GroupAction( #! SetRemap(src, dst) significa: "quando un nodo dentro il GroupAction usa il topic src, usa dst al suo posto".
            actions=[
                # Ricollega l'odometria: nav2 legge /odom, il controller pubblica sul suo topic
                SetRemap(src='/odom', dst='/ackermann_steering_controller/odometry'),
                
                # Ricollega i comandi di velocità
                SetRemap(src='/cmd_vel', dst='/ackermann_steering_controller/reference_unstamped'),
                
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([nav2_bringup_share_dir, '/launch', '/bringup_launch.py']),
                    launch_arguments={
                        'map':          map_yaml_path,
                        'use_sim_time': use_sim_time,
                    }.items(),
                )
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])