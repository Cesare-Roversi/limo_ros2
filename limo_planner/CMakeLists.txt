cmake_minimum_required(VERSION 3.8)
project(limo_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

#Verify#
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
#-----#
#aggiunta action-server
find_package(limo_planner_interfaces REQUIRED)
#-----#

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    nav2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    rclcpp_lifecycle
    lifecycle_msgs
    moveit_ros_planning_interface
    geometry_msgs
    tf2
    tf2_geometry_msgs
    control_msgs
  #aggiunta action-server
    limo_planner_interfaces
    rclcpp_components
  #-----#
)

# LIBRERIA:
add_library(limo_utils
    src/utils/debug.cpp
    src/utils/controller_functions.cpp
    src/utils/parser.cpp
    src/utils/get_user_input.cpp
    src/utils/utils_connections.cpp
    src/utils/utils_global_vars.cpp
    src/utils/utils_objects.cpp
    src/utils/utils_robots.cpp
    src/utils/utils_waypoints.cpp
    src/utils/arm_world_data_utils.cpp
)
ament_target_dependencies(limo_utils ${dependencies})
target_include_directories(limo_utils PUBLIC include)
target_link_libraries(limo_utils yaml-cpp)


# ESEGUIBILI:
#move_action_node
add_executable(move_action_node src/actions/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})
target_link_libraries(move_action_node limo_utils)

#patrol_action_node
add_executable(patrol_action_node src/actions/patrol_action_node.cpp)
ament_target_dependencies(patrol_action_node
  rclcpp
  plansys2_executor
  sensor_msgs
  cv_bridge
  ament_index_cpp
)
target_link_libraries(patrol_action_node ${OpenCV_LIBRARIES})
target_link_libraries(patrol_action_node limo_utils)

#charge_action_node
add_executable(charge_action_node
    src/actions/charge_action_node.cpp
)
ament_target_dependencies(charge_action_node ${dependencies})
target_link_libraries(charge_action_node limo_utils)

#move_arm_action_node
add_executable(move_arm_action_node
    src/actions/move_arm_action_node.cpp
)
ament_target_dependencies(move_arm_action_node ${dependencies})
target_link_libraries(move_arm_action_node limo_utils)

#pick_up_object_action_node
add_executable(pick_up_object_action_node
    src/actions/pick_up_object_action_node.cpp
)
ament_target_dependencies(pick_up_object_action_node ${dependencies})
target_link_libraries(pick_up_object_action_node limo_utils)

#put_down_object_action_node
add_executable(put_down_object_action_node
    src/actions/put_down_object_action_node.cpp
)
ament_target_dependencies(put_down_object_action_node ${dependencies})
target_link_libraries(put_down_object_action_node limo_utils)

#retract_arm_action_node
add_executable(retract_arm_action_node
    src/actions/retract_arm_action_node.cpp
)
ament_target_dependencies(retract_arm_action_node ${dependencies})
target_link_libraries(retract_arm_action_node limo_utils)

#check_distance_action_node
add_executable(check_distance_action_node
    src/actions/check_distance_action_node.cpp
)
ament_target_dependencies(check_distance_action_node ${dependencies})
target_link_libraries(check_distance_action_node limo_utils)

# CONTROLLER
add_executable(controller
    src/controller.cpp
)
ament_target_dependencies(controller ${dependencies})
target_link_libraries(controller limo_utils)

#gui
add_executable(controllerGUIhelper
    src/controllerGUIhelper.cpp
)
target_include_directories(controllerGUIhelper PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(controllerGUIhelper
  PRIVATE "LIMO_PLANNER_CPP_BUILDING_DLL")
ament_target_dependencies(controllerGUIhelper ${dependencies})
target_link_libraries(controllerGUIhelper limo_utils)
#

install(DIRECTORY launch pddl config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  move_action_node
  patrol_action_node
  charge_action_node
  move_arm_action_node
  pick_up_object_action_node
  put_down_object_action_node
  retract_arm_action_node
  check_distance_action_node
  controller
  controllerGUIhelper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/scripts/controllerGUI.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
