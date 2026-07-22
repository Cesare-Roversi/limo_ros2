#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"

void print_world_model(
    rclcpp::Node * node, //serve solo per node->get_logger()
    std::shared_ptr<plansys2::DomainExpertClient> domain,
    std::shared_ptr<plansys2::ProblemExpertClient> problem,
    bool print_structs = false);

void print_pose_stamped(const geometry_msgs::msg::PoseStamped& msg);

