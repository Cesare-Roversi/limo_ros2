import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = LaunchConfiguration('package_name')
    
    base_launch_file = LaunchConfiguration('base_launch_file')
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params')
    # Nuovo parametro per EKF
    ekf_params_file = LaunchConfiguration('ekf_params')


    # DICHIARAZIONE DEGLI ARGOMENTI (VISIBILI DA TERMINALE)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock')

    declare_package_name = DeclareLaunchArgument(
        'package_name', default_value='limo_car',
        description='Nome del pacchetto che contiene launch, config e mappe')

    declare_base_launch_file = DeclareLaunchArgument(
        'base_launch_file', default_value='ackermann_gazebo_povo.launch.py',
        description='Nome del file launch base per spawnare robot e simulatore')

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name), 'maps', 'mappa_povo.yaml'
        ]),
        description='Percorso dalla root del pacchetto al file .yaml della mappa')

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name), 'config', 'nav2_config.yaml'
        ]),
        description='Percorso dalla root del pacchetto al file dei parametri di Nav2')

    # Dichiarazione per il file EKF separato
    declare_ekf_params = DeclareLaunchArgument(
        'ekf_params',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name), 'config', 'ekf_config.yaml'
        ]),
        description='Percorso dalla root del pacchetto al file dei parametri di EKF (robot_localization)')



    #(LAUNCHER DELLA SIMULAZIONE) Inclusione di ackermann_gazebo_*.launch.py
    include_base_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name), 'launch', base_launch_file
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    

    # EKF Node - Ora usa ekf_params_file
    start_ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file, 
            {'use_sim_time': use_sim_time}
        ])

    # Map Server
    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file}
        ])

    # AMCL
    start_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ])

    # Nav2 Lifecycle Manager
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ])



    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_package_name)
    ld.add_action(declare_base_launch_file)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_ekf_params)

    ld.add_action(include_base_robot_launch)
    ld.add_action(start_ekf_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld