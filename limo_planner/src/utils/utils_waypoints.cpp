#include "world_data_utils.hpp"

/*
load_*_from_yaml //salva tutta la mappa nel yaml
save_*_to_yaml   //carica tutto il yaml nella mappa
add_*            //aggiunge * alla mappa -> salva tutta la mappa nel yaml
delete_*         //elimina * dalla mappa -> salva tutta la mappa nel yaml
get_*            //restituisce (in base al nome) una struct o classe C++
*/


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
