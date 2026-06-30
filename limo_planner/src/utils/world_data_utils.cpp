#include "world_data_utils.hpp"

std::string waypoints_filepath_;
std::string objects_filepath_;
std::string robots_filepath_;

std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> map_waypoints;
std::unordered_map<std::string, Object> map_objects;
std::unordered_map<std::string, Robot> map_robots;


//* WAYPOINTS
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


void load_waypoints_from_yaml(){
    map_waypoints.clear();
    YAML::Node config = YAML::LoadFile(waypoints_filepath_);
    if (config["waypoints"]) {
        for (const auto & wp : config["waypoints"]) {
            std::string name = wp.first.as<std::string>();
            double x = wp.second["x"].as<double>();
            double y = wp.second["y"].as<double>();
            double yaw = wp.second["yaw"].as<double>();
            map_waypoints[name] = make_waypoint(x, y, yaw);
        }
    }
}


void print_waypoints(){
    cout << "PRINT_WAYPOINTS" << endl;

    for (const auto& [name, wp] : map_waypoints) {
        std::cout << name << " -> ("
                  << wp.pose.position.x << ", "
                  << wp.pose.position.y << ", "
                  << wp.pose.position.z << ")\n";
    }
}

void save_waypoints_to_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "waypoints" << YAML::Value << YAML::BeginMap;
    for (const auto & [name, pose] : map_waypoints) {
        double yaw = std::atan2(
            2.0 * (pose.pose.orientation.w * pose.pose.orientation.z),
            1.0 - 2.0 * (pose.pose.orientation.z * pose.pose.orientation.z));
        out << YAML::Key << name << YAML::Value << YAML::BeginMap
            << YAML::Key << "x" << YAML::Value << pose.pose.position.x
            << YAML::Key << "y" << YAML::Value << pose.pose.position.y
            << YAML::Key << "yaw" << YAML::Value << yaw
            << YAML::EndMap;
    }
    out << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(waypoints_filepath_);
    fout << out.c_str();
}

void clear_waypoints_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "waypoints" << YAML::Value << YAML::BeginMap
        << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(waypoints_filepath_);
    fout << out.c_str();
}

void clear_waypoints_map(){
    map_waypoints.clear();
}

void clear_waypoints(){
    clear_waypoints_map();
    clear_waypoints_yaml();
}

void add_waypoint(
    const std::string & name,
    double x, double y, double yaw,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    map_waypoints[name] = make_waypoint(x, y, yaw);
    save_waypoints_to_yaml();
    cout << name << " -> " << problem->addInstance(plansys2::Instance{name, "waypoint"}) << endl;
}


void delete_waypoint(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem)
{
    load_waypoints_from_yaml();

    auto it = map_waypoints.find(name);
    if (it == map_waypoints.end()) {
        cout << "ERROR: Waypoint '" << name << "' not found in map_waypoints" << endl;
        return;
    }

    map_waypoints.erase(it);
    save_waypoints_to_yaml();

    cout << name << " -> " << problem->removeInstance(plansys2::Instance{name, "waypoint"}) << endl;
}


geometry_msgs::msg::PoseStamped get_waypoint(const std::string & name){
    load_waypoints_from_yaml();
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

//* __WAYPOINTS


//* OBJECTS
void load_objects_from_yaml(){
    map_objects.clear();
    YAML::Node config = YAML::LoadFile(objects_filepath_);
    if (config["objects"]) {
        for (const auto & obj : config["objects"]) {
            std::string name = obj.first.as<std::string>();
            float height = obj.second["height"].as<float>();
            float max_width = obj.second["max_width"].as<float>();
            float min_width = obj.second["min_width"].as<float>();
            float weight = obj.second["weight"].as<float>();
            map_objects[name] = Object(height, max_width, min_width, weight);
        }
    }
}

void print_objects(){
    cout << "PRINT_OBJECTS" << endl;

    for (const auto& [name, obj] : map_objects) {
        std::cout << name << " -> ("
                  << obj.height << ", "
                  << obj.max_width << ", "
                  << obj.min_width << ", "
                  << obj.weight << ")\n";
    }
}

void save_objects_to_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "objects" << YAML::Value << YAML::BeginMap;
    for (const auto & [name, obj] : map_objects) {
        out << YAML::Key << name << YAML::Value << YAML::BeginMap
            << YAML::Key << "height" << YAML::Value << obj.height
            << YAML::Key << "max_width" << YAML::Value << obj.max_width
            << YAML::Key << "min_width" << YAML::Value << obj.min_width
            << YAML::Key << "weight" << YAML::Value << obj.weight
            << YAML::EndMap;
    }
    out << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(objects_filepath_);
    fout << out.c_str();
}

void clear_objects_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "objects" << YAML::Value << YAML::BeginMap
        << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(objects_filepath_);
    fout << out.c_str();
}

void clear_objects_map(){
    map_objects.clear();
}

void clear_objects(){
    clear_objects_map();
    clear_objects_yaml();
}

void add_object(
    const std::string & name,
    float height, float max_width, float min_width, float weight,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    map_objects[name] = Object(height, max_width, min_width, weight);
    save_objects_to_yaml();
    cout << name << " -> " << problem->addInstance(plansys2::Instance{name, "object"}) << endl;
}

void delete_object(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem)
{
    load_objects_from_yaml();

    auto it = map_objects.find(name);
    if (it == map_objects.end()) {
        cout << "ERROR: Object '" << name << "' not found in map_objects" << endl;
        return;
    }

    map_objects.erase(it);
    save_objects_to_yaml();

    cout << name << " -> " << problem->removeInstance(plansys2::Instance{name, "object"}) << endl;
}

Object get_object(const std::string & name){
    load_objects_from_yaml();
    auto it = map_objects.find(name);
    if (it == map_objects.end()) {
        throw std::runtime_error("ERROR: Object '" + name + "' not found in map_objects");
    }
    return it->second;
}

std::string get_object_str(const std::string & name){
    auto obj = get_object(name);

    std::ostringstream ss;
    ss << name << ":\n";
    ss << std::fixed << std::setprecision(3);
    ss << "        height: " << obj.height << "\n";
    ss << "        max_width: " << obj.max_width << "\n";
    ss << "        min_width: " << obj.min_width << "\n";
    ss << "        weight: " << obj.weight << "\n";
    return ss.str();
}
//* __OBJECTS



//* ROBOTS
void load_robots_from_yaml(){
    map_robots.clear();
    YAML::Node config = YAML::LoadFile(robots_filepath_);
    if (config["robots"]) {
        for (const auto & rb : config["robots"]) {
            std::string name = rb.first.as<std::string>();
            float battery_voltage = rb.second["battery_voltage"].as<float>();
            float battery_mah = rb.second["battery_mah"].as<float>();
            float motor_power = rb.second["motor_power"].as<float>();
            float max_robot_velocity = rb.second["max_robot_velocity"].as<float>();
            map_robots[name] = Robot(battery_voltage, battery_mah, motor_power, max_robot_velocity);
        }
    }
}

void print_robots(){
    cout << "PRINT_ROBOTS" << endl;

    for (const auto& [name, rb] : map_robots) {
        std::cout << name << " -> ("
                  << rb.battery_voltage << ", "
                  << rb.battery_mah << ", "
                  << rb.motor_power << ", "
                  << rb.max_robot_velocity << ")\n";
    }
}

void save_robots_to_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "robots" << YAML::Value << YAML::BeginMap;
    for (const auto & [name, rb] : map_robots) {
        out << YAML::Key << name << YAML::Value << YAML::BeginMap
            << YAML::Key << "battery_voltage" << YAML::Value << rb.battery_voltage
            << YAML::Key << "battery_mah" << YAML::Value << rb.battery_mah
            << YAML::Key << "motor_power" << YAML::Value << rb.motor_power
            << YAML::Key << "max_robot_velocity" << YAML::Value << rb.max_robot_velocity
            << YAML::EndMap;
    }
    out << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(robots_filepath_);
    fout << out.c_str();
}

void clear_robots_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "robots" << YAML::Value << YAML::BeginMap
        << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(robots_filepath_);
    fout << out.c_str();
}

void clear_robots_map(){
    map_robots.clear();
}

void clear_robots(){
    clear_robots_map();
    clear_robots_yaml();
}

void add_robot(
    const std::string & name,
    float battery_voltage, float battery_mah, float motor_power, float max_robot_velocity,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    map_robots[name] = Robot(battery_voltage, battery_mah, motor_power, max_robot_velocity);
    save_robots_to_yaml();
    cout << name << " -> " << problem->addInstance(plansys2::Instance{name, "robot"}) << endl;
}

void delete_robot(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem)
{
    load_robots_from_yaml();

    auto it = map_robots.find(name);
    if (it == map_robots.end()) {
        cout << "ERROR: Robot '" << name << "' not found in map_robots" << endl;
        return;
    }

    map_robots.erase(it);
    save_robots_to_yaml();

    cout << name << " -> " << problem->removeInstance(plansys2::Instance{name, "robot"}) << endl;
}

Robot get_robot(const std::string & name){
    load_robots_from_yaml();
    auto it = map_robots.find(name);
    if (it == map_robots.end()) {
        throw std::runtime_error("ERROR: Robot '" + name + "' not found in map_robots");
    }
    return it->second;
}

std::string get_robot_str(const std::string & name){
    auto rb = get_robot(name);

    std::ostringstream ss;
    ss << name << ":\n";
    ss << std::fixed << std::setprecision(3);
    ss << "        battery_voltage: " << rb.battery_voltage << "\n";
    ss << "        battery_mah: " << rb.battery_mah << "\n";
    ss << "        motor_power: " << rb.motor_power << "\n";
    ss << "        max_robot_velocity: " << rb.max_robot_velocity << "\n";
    return ss.str();
}
//*__ROBOTS



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