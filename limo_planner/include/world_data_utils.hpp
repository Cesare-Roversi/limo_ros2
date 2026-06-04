#pragma once
#include <unordered_map>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <plansys2_pddl_parser/Utils.h>
#include <memory>
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "world_data_structs.hpp"

extern std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> map_waypoints;
extern std::unordered_map<std::string, Object> map_objects;
extern std::unordered_map<std::string, Robot> map_robots;

geometry_msgs::msg::PoseStamped make_waypoint(float x, float y, float yaw);
void add_waypoint(
    const std::string & name,
    double x, double y, double yaw,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);
void remove_waypoint(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);
void modify_waypoint(
    const std::string & name,
    double x, double y, double yaw);
geometry_msgs::msg::PoseStamped get_waypoint(const std::string & name);

std::string get_waypoint_str(const std::string & name);

void add_object(
    const std::string & name,
    float height, float max_width, float min_width, float weight,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);
void remove_object(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);
void modify_object(
    const std::string & name,
    float height, float max_width, float min_width, float weight);
Object get_object(const std::string & name);

std::string get_object_str(const std::string & name);

// ROBOTS:
void add_robot(
    const std::string & name,
    float battery_voltage, float battery_mah, float motor_power,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);
void remove_robot(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);
void modify_robot(
    const std::string & name,
    float battery_voltage, float battery_mah, float motor_power);
Robot get_robot(const std::string & name);

std::string get_robot_str(const std::string & name);


// RAGGRUPPAMENTO:
std::string get_instance_str(const std::string& type, const std::string& name);