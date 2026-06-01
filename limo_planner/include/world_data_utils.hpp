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

extern std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> map_waypoints;

geometry_msgs::msg::PoseStamped make_waypoint(float x, float y, float yaw);
void add_waypoint(
    const std::string & name,
    double x, double y, double yaw,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);