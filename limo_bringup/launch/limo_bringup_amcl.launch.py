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
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ARGOMENTI:
    use_sim_time_dec = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    map_file_dec = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('limo_description'), 'maps/mappa_povo', 'povo.yaml'
        ]),
        description='Full path to map yaml file to load'
    )
    
    nav2_params_file_dec = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('limo_bringup'), 'param', 'tesi_nav2_ackermann.yaml'
        ]),
        description='Full path to nav2 param file to load'
    )
    # ESISTEVANO un file UGUALI ma con: nav2.yaml, navigation2.yaml

    rviz2_config_file_dec = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz' 
        ]),
        description='Full path to rviz config file to load'
    )


    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz2_config_file = LaunchConfiguration('rviz_config_file')
    # __ARGOMENTI



    
    
    # Relay /tf_odometry -> /tf per avere l'albero TF completo (odom->base_link unito a base_link->resto)
    tf_odom_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='tf_odom_relay_node',
        arguments=['/ackermann_steering_controller/tf_odometry', '/tf'], #! TENIAMO /tf (assumiamo namespace globale)
        parameters=[{'use_sim_time': use_sim_time}], 
        output='screen'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map':          map_file,
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
        }.items(),
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    group_action = GroupAction( #! SetRemap(src, dst) significa: "quando un nodo dentro il GroupAction usa il topic src, usa dst al suo posto".
        actions=[
            # Ricollega l'odometria: nav2 legge /odom, il controller pubblica sul suo topic
            SetRemap(src='/odom', dst='/ackermann_steering_controller/odometry'),
            # Ricollega i comandi di velocità
            SetRemap(src='/cmd_vel', dst='/ackermann_steering_controller/reference_unstamped'),
            
            nav2_launch,
            rviz2_node
        ]
    )


    return LaunchDescription([
        use_sim_time_dec,
        map_file_dec,
        nav2_params_file_dec,
        rviz2_config_file_dec,
        tf_odom_relay_node,
        group_action
    ])