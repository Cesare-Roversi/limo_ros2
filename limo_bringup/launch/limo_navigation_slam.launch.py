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

    # ARGOMENTI:
    use_sim_time_dec = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    nav2_params_file_dec = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            # FindPackageShare('limo_bringup'), 'param', 'tesi_nav2_ackermann.yaml'
            FindPackageShare('limo_bringup'), 'param', 'PROVA01_nav2_amcl.yaml'
        ])
    )

    slam_params_file_dec = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('limo_bringup'), 'param', 'slam_toolbox_params.yaml'
        ])
    )

    rviz2_config_file_dec = DeclareLaunchArgument(
        'rviz2_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'
        ])
    )


    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz2_config_file = LaunchConfiguration('rviz2_config_file')

    # __ARGOMENTI


    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
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
            'params_file': nav2_params_file,
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    tf_odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='tf_odom_relay',
        arguments=['/ackermann_steering_controller/tf_odometry', '/tf'], #! TENIAMO /tf (assumiamo namespace globale)
        parameters=[{'use_sim_time': True}], 
        output='screen'
    )

    group_action = GroupAction(
        actions=[
            SetRemap(src='/odom', dst='/ackermann_steering_controller/odometry'),
            SetRemap(src='/cmd_vel', dst='/ackermann_steering_controller/reference_unstamped'),
            slam_toolbox_node,
            nav2_launch,
            rviz2_node,
        ]
    )

    return LaunchDescription([
        use_sim_time_dec,
        nav2_params_file_dec,
        slam_params_file_dec,
        rviz2_config_file_dec,
        tf_odom_relay,
        group_action,
    ])