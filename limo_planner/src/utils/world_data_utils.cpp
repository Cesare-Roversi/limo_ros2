#include "world_data_utils.hpp"
#include <cmath>
using namespace std;

// Una mappa per ogni entity type del world model
std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> map_waypoints;
std::unordered_map<std::string, Object> map_objects;



//* WAYPOINTS:
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


void remove_waypoint(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    // Remove from the map
    auto it = map_waypoints.find(name);
    if (it != map_waypoints.end()) {
        map_waypoints.erase(it);
    }
    
    // Remove from problem expert
    bool result = problem->removeInstance(plansys2::Instance{name, "waypoint"});
    cout << "Removed waypoint: " << name << " -> " << result << endl;
}


void modify_waypoint(
    const std::string & name,
    double x, double y, double yaw){

    // Check if waypoint exists
    auto it = map_waypoints.find(name);
    if (it == map_waypoints.end()) {
        throw std::runtime_error("ERROR: Waypoint '" + name + "' not found in map_waypoints");
    }
    
    // Update the waypoint
    map_waypoints[name] = make_waypoint(x, y, yaw);
    cout << "Modified waypoint: " << name << endl;
}


geometry_msgs::msg::PoseStamped get_waypoint(const std::string & name){
    
    // Check if waypoint exists
    auto it = map_waypoints.find(name);
    if (it == map_waypoints.end()) {
        throw std::runtime_error("ERROR: Waypoint '" + name + "' not found in map_waypoints");
    }
    
    return it->second;
}


std::string get_waypoint_str(const std::string & name){
    auto waypoint = get_waypoint(name);
    const auto & p = waypoint.pose.position;
    const auto & q = waypoint.pose.orientation;

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    std::ostringstream ss;
    ss << name << ":\n";
    ss << "        frame_id: " << waypoint.header.frame_id << "\n";
    ss << std::fixed << std::setprecision(3);
    ss << "        x: " << p.x << "\n";
    ss << "        y: " << p.y << "\n";
    ss << "        z: " << p.z << "\n";
    ss << "        yaw: " << yaw << "\n";
    return ss.str();
}


//*OBJECTS:
void add_object(
    const std::string & name,
    float height, float max_width, float min_width, float weight,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    map_objects[name] = Object(height, max_width, min_width, weight);
    cout << "Added object: " << name << " -> " << problem->addInstance(plansys2::Instance{name, "object"}) << endl;
}


void remove_object(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    // Remove from the map
    auto it = map_objects.find(name);
    if (it != map_objects.end()) {
        map_objects.erase(it);
    }
    
    // Remove from problem expert
    bool result = problem->removeInstance(plansys2::Instance{name, "object"});
    cout << "Removed object: " << name << " -> " << result << endl;
}


void modify_object(
    const std::string & name,
    float height, float max_width, float min_width, float weight){

    // Check if object exists
    auto it = map_objects.find(name);
    if (it == map_objects.end()) {
        throw std::runtime_error("ERROR: Object '" + name + "' not found in map_objects");
    }
    
    // Update the object
    map_objects[name] = Object(height, max_width, min_width, weight);
    cout << "Modified object: " << name << endl;
}


Object get_object(const std::string & name){
    
    // Check if object exists
    auto it = map_objects.find(name);
    if (it == map_objects.end()) {
        throw std::runtime_error("ERROR: Object '" + name + "' not found in map_objects");
    }
    
    return it->second;
}


std::string get_object_str(const std::string & name){
    auto object = get_object(name);
    std::ostringstream ss;
    ss << name << ":\n";
    ss << std::fixed << std::setprecision(3);
    ss << "        height: " << object.height << "\n";
    ss << "        max_width: " << object.max_width << "\n";
    ss << "        min_width: " << object.min_width << "\n";
    ss << "        weight: " << object.weight << "\n";
    return ss.str();
}


// RAGGRUPPAMENTO:
std::string get_instance_str(const std::string& type, const std::string& name) {
    if (type == "waypoint") {
        return get_waypoint_str(name);
    } 
    else if (type == "object") {
        return get_object_str(name);
    } 
    else {
        return "WARNING: TYPE: [" + type + "] NOT FOUND\n";
    }
}