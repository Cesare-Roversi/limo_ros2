import os
import shutil

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO

from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable

#! è molto simile a ackermann_gazebo.launch.py, questo sarà in nostro principale


def generate_launch_description():

    has_desktop_environment = os.environ.get('XDG_CURRENT_DESKTOP', 'NONE') != 'NONE'
    has_gnome_terminal = shutil.which('gnome-terminal') is not None

    # Get share directories of packages:
    package_name = 'limo_car'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_limo_description = get_package_share_directory('limo_description')
    
    # Get default files from shared directories of packages:
    default_world_path = os.path.join(pkg_limo_description, 'worlds', 'world_povo.sdf')



    # PARAMETRI:
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path_cfg = LaunchConfiguration('world_path')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    jsb_delay_val = LaunchConfiguration('jsb_delay')
    bridge_rviz_delay_val = LaunchConfiguration('bridge_rviz_delay')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_world_path = DeclareLaunchArgument('world_path', default_value=default_world_path)
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='0.0')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='0.0')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.65')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    declare_start_rviz = DeclareLaunchArgument('start_rviz', default_value='false')
    declare_rviz_config = DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg_path, 'config', 'limo_visual.rviz'))
    declare_jsb_delay = DeclareLaunchArgument('jsb_delay', default_value='20.0')
    declare_bridge_rviz_delay = DeclareLaunchArgument('bridge_rviz_delay', default_value='10.0')
    


    # inizializza il robot simulato
    mbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch', 'ackermann.launch.py')]), 
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [world_path_cfg, ' -r'],
            'on_exit_shutdown': 'True',
            'paused': 'False',
            'use_sim_time': use_sim_time
        }.items(),
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=pkg_limo_description
    )
    
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description', '-entity', 'mbot',
                                   '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw],
                        output='screen')
    
    robot_controllers = PathJoinSubstitution([pkg_path, 'config', 'ackermann_drive_controller.yaml'])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'],
    )
    
    ackermann_steering_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['ackermann_steering_controller', '--param-file', robot_controllers],
    )

    delayed_joint_state_broadcaster_spawner = TimerAction(period=jsb_delay_val, actions=[joint_state_broadcaster_spawner])
    
    prefix_value = 'gnome-terminal --tab --' if (has_desktop_environment and has_gnome_terminal) else ''

    ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_path, 'config', 'ros_gz_bridge.yaml')}],
        prefix=prefix_value, output='screen'
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world_path,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_rviz_config,
        declare_start_rviz,
        declare_jsb_delay,
        declare_bridge_rviz_delay,
        mbot,
        set_gazebo_resource_path,
        gz_sim,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[delayed_joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_steering_controller_spawner],
            )
        ),
        TimerAction(period=bridge_rviz_delay_val, actions=[ros_gz_bridge, rviz_node])
    ])