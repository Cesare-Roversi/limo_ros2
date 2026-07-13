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

    share_limo_description = get_package_share_directory('limo_description')
    share_nav2_bringup = get_package_share_directory('nav2_bringup')
    share_limo_bringup = get_package_share_directory('limo_bringup')

    # ARGOMENTI:
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_file_path = DeclareLaunchArgument(
        'map_file_path',
        default_value=PathJoinSubstitution([
            share_limo_description, 'maps/mappa_povo', 'povo.yaml'
        ]),
        description='Full path to map yaml file to load'
    )
    
    declare_nav2_params_file_path = DeclareLaunchArgument(
        'nav2_params_file_path',
        default_value=PathJoinSubstitution([
            # share_limo_bringup, 'config', 'tesi_nav2_ackermann.yaml'
            # share_limo_bringup, 'config', 'PROVA01_nav2_amcl.yaml'
            share_limo_bringup, 'config', 'PROVA02_nav2_amcl.yaml'
        ]),
        description='Full path to nav2 param file to load'
    )
    

    declare_rviz2_config_file_path = DeclareLaunchArgument(
        'rviz_config_file_path',
        default_value=PathJoinSubstitution([
            share_limo_bringup, 'rviz', 'nav2_default_view.rviz' 
        ]),
        description='Full path to rviz config file to load'
    )


    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file_path = LaunchConfiguration('map_file_path')
    nav2_params_file_path = LaunchConfiguration('nav2_params_file_path')
    rviz2_config_file_path = LaunchConfiguration('rviz_config_file_path')
    # __ARGOMENTI



    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                share_nav2_bringup,
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map':          map_file_path,
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file_path,
        }.items(),
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_file_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )


    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_id': 'base_link'}
        ],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),        # prima era /cmd_vel_unstamped
            ('cmd_vel_out', '/cmd_vel_stamped') # nuovo topic dedicato
        ],
        output='screen'
    )


    group_action = GroupAction( #! SetRemap(src, dst) significa: "quando un nodo dentro il GroupAction usa il topic src, usa dst al suo posto".
        actions=[
            # Ricollega l'odometria: nav2 legge /odom, il controller pubblica sul suo topic
            SetRemap(src='/odom', dst='/odometry/filtered'),
            
            nav2_launch,
            rviz2_node
        ]
    )


    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file_path,
        declare_nav2_params_file_path,
        declare_rviz2_config_file_path,
        twist_stamper_node,
        group_action
    ])