import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution 
from launch_ros.actions import Node, SetRemap                                
from launch_ros.substitutions import FindPackageShare                        
from launch.actions import GroupAction


def generate_launch_description():

    # SHARE DIRS:
    share_limo_bringup = get_package_share_directory('limo_bringup')
    share_nav2_bringup = get_package_share_directory('nav2_bringup')


    # ARGOMENTI:
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file_path = LaunchConfiguration('nav2_params_file_path')
    slam_params_file_path = LaunchConfiguration('slam_params_file_path')
    rviz2_config_file_path = LaunchConfiguration('rviz2_config_file_path')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_nav2_params_file_path = DeclareLaunchArgument(
        'nav2_params_file_path',
        default_value=PathJoinSubstitution([
            # FindPackageShare('limo_bringup'), 'param', 'tesi_nav2_ackermann.yaml'
            share_limo_bringup, 'config', 'ackermann_slam.yaml'
        ])
    )

    declare_slam_params_file_path = DeclareLaunchArgument(
        'slam_params_file_path',
        default_value=PathJoinSubstitution([
            share_limo_bringup, 'config', 'slam_toolbox_params.yaml'
        ])
    )

    declare_rviz2_config_file_path = DeclareLaunchArgument(
        'rviz2_config_file_path',
        default_value=PathJoinSubstitution([
            share_nav2_bringup, 'rviz', 'nav2_default_view.rviz'
        ])
    )
    # __ARGOMENTI




    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('limo_bringup'),
                'launch',
                'limo_slam_toolbox.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file_path': slam_params_file_path 
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file_path,
        }.items()
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
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/cmd_vel_stamped')
        ],
        output='screen'
    )


    group_action = GroupAction(
        actions=[
            SetRemap(src='/odom', dst='/odometry/filtered'),
            # SetRemap(src='/cmd_vel', dst='/ackermann_steering_controller/reference_unstamped'),
            slam_toolbox_launch,
            nav2_launch,
            rviz2_node,
        ]
    )


    return LaunchDescription([
        declare_use_sim_time,
        declare_nav2_params_file_path,
        declare_slam_params_file_path,
        declare_rviz2_config_file_path,
        twist_stamper_node,
        group_action,
    ])