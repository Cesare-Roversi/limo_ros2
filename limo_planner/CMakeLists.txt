cmake_minimum_required(VERSION 3.8)
project(limo_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

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
)

# LIBRERIA:
add_library(limo_utils
    src/utils/debug.cpp
    src/utils/world_data_utils.cpp
)
ament_target_dependencies(limo_utils ${dependencies})
target_include_directories(limo_utils PUBLIC include)


# ESEGUIBILI:
add_executable(move_action_node src/actions/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})
target_link_libraries(move_action_node limo_utils)

add_executable(controller
    src/controller.cpp
)
ament_target_dependencies(controller ${dependencies})
target_link_libraries(controller limo_utils)



install(DIRECTORY launch pddl
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  move_action_node
  controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
