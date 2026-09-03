import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    share_limo_planner = get_package_share_directory('limo_planner')
    GUI_node = Node(
        package='limo_planner',
        executable='controllerGUI.py',
        name='GUI_node',
        output='screen',
        parameters=[]
    )
    controllerGUIhelper_node = Node(
        package='limo_planner',
        executable='controllerGUIhelper',
        name='controllerGUIhelper_node',
        output='screen',
        parameters=[]
    )
    #plansys2_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([os.path.join(share_limo_planner,'launch', 'launch_default.launch.py')]),
    #    launch_arguments={
    #        'use_gui': 'true'
    #    }.items()
    #)

    return LaunchDescription([
        #plansys2_launch,
        controllerGUIhelper_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controllerGUIhelper_node,
                on_start=[GUI_node],
            )
        ),
    ])