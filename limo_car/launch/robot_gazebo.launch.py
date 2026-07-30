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
from launch.actions import LogInfo

from launch.actions import AppendEnvironmentVariable
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    share_limo_car = get_package_share_directory('limo_car')
    share_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    share_limo_description = get_package_share_directory('limo_description')
    
    # Get default files from shared directories of packages:
    default_world_path = os.path.join(share_limo_description, 'worlds', 'world_povo.sdf')


    # PARAMETRI:
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path_cfg = LaunchConfiguration('world_path')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    start_rviz_gazebo = LaunchConfiguration('start_rviz_gazebo')
    rviz_config = LaunchConfiguration('rviz_config')
    ekf_node_config = LaunchConfiguration('ekf_node_config')
    controller_type = LaunchConfiguration('controller_type')



    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_world_path = DeclareLaunchArgument('world_path', default_value=default_world_path)
    #declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='120.0')
    #declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='32.0')
    #declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.6')
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='0.804443')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='2.446670')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.6')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='-1.57')
    declare_start_rviz_gazebo = DeclareLaunchArgument('start_rviz_gazebo', default_value='false')
    declare_rviz_config = DeclareLaunchArgument('rviz_config', default_value=os.path.join(share_limo_car, 'config', 'limo_visual.rviz'))
    declare_ekf_node_config = DeclareLaunchArgument(
        'ekf_node_config',
        default_value=PathJoinSubstitution([share_limo_car, 'config', 'ekf_config_ackermann.yaml'])
    )
    declare_controller_type =  DeclareLaunchArgument(
        'controller_type',
        default_value ='ackermann_steering_controller',
        description ='controller_type can be set to: ackermann_steering_controller OR diff_drive_controller (THE NAMES IN robot_controllers.yaml)'
    )
    
    # _PARAMETRI


    #! DELAY:
    jsb_delay_val = 20.0
    bridge_rviz_delay_val = 10.0


    # INIZIALIZZO ROBOT
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(share_limo_car,'launch', 'robot.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'controller_type': controller_type,
            'start_node_state_publisher': 'false',
            'start_rviz': 'false'
            }.items()
    )

    # INIZIALIZZO SIMULAZIONE GAZEBO
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [world_path_cfg, ' -r'],
            'on_exit_shutdown': 'True',
            'paused': 'False',
            'use_sim_time': use_sim_time
        }.items(),
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=share_limo_description
    )

    #aggiungo la path per il mycobot
    mycobot_description_path = get_package_share_directory('mycobot_description')
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[mycobot_description_path + '/../']
    )
    
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description', '-entity', 'mbot',
                                   '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw],
                        output='screen')
    
    

    robot_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[controller_type], 
    )
    

    # Aspetta che la simulzione sia avviata prima di far partire il joint_state_broadcaster_spawner:
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'],
    )
    delayed_joint_state_broadcaster_spawner = TimerAction(period=jsb_delay_val, actions=[joint_state_broadcaster_spawner])



    #Materiale per moveit2 e controller del braccio:
    urdf_absolute_path = os.path.join(
        share_limo_description, "urdf", "limo_mycobot.xacro.urdf"
    )
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="mycobot_280_moveit2")
        .robot_description(file_path=urdf_absolute_path)
        .robot_description_semantic(file_path="config/firefighter.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # Se può lanciare il bridge in una nuova finestra di terminale lo fa:
    has_desktop_environment = os.environ.get('XDG_CURRENT_DESKTOP', 'NONE') != 'NONE'
    has_gnome_terminal = shutil.which('gnome-terminal') is not None
    prefix_value = 'gnome-terminal --tab --' if (has_desktop_environment and has_gnome_terminal) else ''
    ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(share_limo_car, 'config', 'ros_gz_bridge.yaml')}],
        prefix=prefix_value, output='screen'
    )


    # ekf node locale
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_node_config, {'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(start_rviz_gazebo)
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--disable-stdout-logs'],
    )


    return LaunchDescription([
        set_gz_resource_path,
        declare_use_sim_time,
        declare_world_path,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_rviz_config,
        declare_start_rviz_gazebo,
        declare_ekf_node_config,
        declare_controller_type,
        robot_launch,
        gz_sim_launch,
        spawn_entity,
        robot_localization_node,
        move_group_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[delayed_joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[gripper_controller_spawner],
            )
        ),
        TimerAction(period=bridge_rviz_delay_val, actions=[ros_gz_bridge, rviz_node]),

        LogInfo(msg=['[DEBUG] start_rviz_gazebo value: ', start_rviz_gazebo])
    ])