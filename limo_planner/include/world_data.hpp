#pragma once
#include <unordered_map>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"

extern std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> waypoints;
