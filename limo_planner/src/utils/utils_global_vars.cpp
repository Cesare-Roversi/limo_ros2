#include "world_data_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


std::string waypoints_filepath_ = ament_index_cpp::get_package_share_directory("limo_planner") + "/config/waypoints.yaml";
std::string objects_filepath_ = ament_index_cpp::get_package_share_directory("limo_planner") + "/config/objects.yaml";
std::string robots_filepath_ = ament_index_cpp::get_package_share_directory("limo_planner") + "/config/robots.yaml";
std::string connections_filepath_ = ament_index_cpp::get_package_share_directory("limo_planner") + "/config/connections.yaml";

std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> map_waypoints;
std::unordered_map<std::string, Object> map_objects;
std::unordered_map<std::string, Robot> map_robots;
std::map<std::string, Connection> map_connections;



//* RAGGRUPPAMENTO:
std::string get_instance_str(const std::string& type, const std::string& name) {
    if (type == "waypoint") {
        return get_waypoint_str(name);
    } 
    else if (type == "object") {
        return get_object_str(name);
    } 
    else if (type == "robot") {
        return get_robot_str(name);
    } 
    else {
        return "WARNING: TYPE: [" + type + "] NOT FOUND\n";
    }
}


void clear_all_yaml(){
    clear_waypoints_yaml();
    clear_objects_yaml();
    clear_robots_yaml();
}

void clear_all_map(){
    clear_waypoints_map();
    clear_objects_map();
    clear_robots_map();
}

void clear_all(){
    clear_waypoints();
    clear_objects();
    clear_robots();
}