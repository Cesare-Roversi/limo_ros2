#include "world_data_utils.hpp"
using namespace std;

// Una mappa per ogni entity type del world model
std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> map_waypoints;




geometry_msgs::msg::PoseStamped make_waypoint(float x, float y, float yaw){
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = rclcpp::Clock().now();
    wp.pose.position.x = x;
    wp.pose.position.y = y;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = std::sin(yaw / 2.0);
    wp.pose.orientation.w = std::cos(yaw / 2.0);
    return wp;
}


void add_waypoint(
    const std::string & name,
    double x, double y, double yaw,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    map_waypoints[name] = make_waypoint(x, y, yaw);
    cout << name << " -> " << problem->addInstance(plansys2::Instance{name, "waypoint"}) << endl;
}
