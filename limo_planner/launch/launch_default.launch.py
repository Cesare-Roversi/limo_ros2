import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#! ATTENTO!! NODI COMMENTATI

def generate_launch_description():
    share_limo_planner = get_package_share_directory('limo_planner')
    share_plansys2_bringup = get_package_share_directory('plansys2_bringup')

    pddl_domain_file = os.path.join(
        share_limo_planner,
        'pddl',
        'domain.pddl' # ! ATTENTO
        #'simple_domain_temp.pddl'
    )

    plansys2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            share_plansys2_bringup,
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': pddl_domain_file
            }.items()
        )

    move_action_node = Node(
        package='limo_planner',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters=[]
    )
    
    patrol_action_node = Node(
        package='limo_planner',
        executable='patrol_action_node',
        name='patrol_action_node',
        output='screen',
        parameters=[]
    )

    charge_action_node = Node(
        package='limo_planner',
        executable='charge_action_node',
        name='charge_action_node',
        output='screen',
        parameters=[]
    )

    move_arm_action_node = Node(
        package='limo_planner',
        executable='move_arm_action_node',
        name='move_arm_action_node',
        output='screen',
        parameters=[]
    )

    pick_up_object_action_node = Node(
        package='limo_planner',
        executable='pick_up_object_action_node',
        name='pick_up_object_action_node',
        output='screen',
        parameters=[]
    )

    put_down_object_action_node = Node(
        package='limo_planner',
        executable='put_down_object_action_node',
        name='put_down_object_action_node',
        output='screen',
        parameters=[]
    )

    retract_arm_action_node = Node(
        package='limo_planner',
        executable='retract_arm_action_node',
        name='retract_arm_action_node',
        output='screen',
        parameters=[]
    )

    check_distance_action_node = Node(
        package='limo_planner',
        executable='check_distance_action_node',
        name='check_distance_action_node',
        output='screen',
        parameters=[]
    )

    controller_node = Node(
        package='limo_planner',
        executable='controller',
        # name='controller',
        output='screen',
        parameters=[]
    )


    return LaunchDescription([
        plansys2_bringup_launch,
        move_action_node,
        patrol_action_node,
        charge_action_node,
        move_arm_action_node,
        pick_up_object_action_node,
        put_down_object_action_node,
        retract_arm_action_node,
        check_distance_action_node,
        # move_with_object_cmd_node,
        # pick_cmd_node,
        # unload_cmd_node,
        # arm_move_cmd_node,
        controller_node
    ])

