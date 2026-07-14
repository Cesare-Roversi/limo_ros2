#include "arm_world_data_utils.hpp"

std::string arm_positions_filepath_;

std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> arm_positions;

/*
load_*_from_yaml //salva tutta la mappa nel yaml
save_*_to_yaml   //carica tutto il yaml nella mappa
add_*            //aggiunge * alla mappa -> salva tutta la mappa nel yaml
delete_*         //elimina * dalla mappa -> salva tutta la mappa nel yaml
get_*            //restituisce (in base al nome) una struct o classe C++
*/


//* ARM POSITIONS

geometry_msgs::msg::PoseStamped make_arm_position(float x, float y, float z, float roll, float pitch, float yaw){
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "arm_workspace";
    p.header.stamp = rclcpp::Clock().now();
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    p.pose.orientation = tf2::toMsg(q);
    return p;
}


void load_arm_positions_from_yaml(){
    arm_positions.clear();
    YAML::Node config = YAML::LoadFile(arm_positions_filepath_);
    if (config["arm_positions"]) {
        for (const auto & p : config["arm_positions"]) {
            std::string name = p.first.as<std::string>();
            double x = p.second["x"].as<double>();
            double y = p.second["y"].as<double>();
            double z = p.second["z"].as<double>();
            double roll = p.second["roll"].as<double>();
            double pitch = p.second["pitch"].as<double>();
            double yaw = p.second["yaw"].as<double>();
            arm_positions[name] = make_arm_position(x, y, z, roll, pitch, yaw);
        }
    }
}



void print_arm_positions(){
    cout << "PRINT_ARM_POSITIONS" << endl;
    for (const auto& [name, p] : arm_positions) {
        auto & q = p.pose.orientation;
        double roll, pitch, yaw;
        tf2::Quaternion quat= tf2::Quaternion(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3(tf2::Matrix3x3(quat)).getRPY(roll, pitch, yaw);
        std::cout << name << " -> ("
                  << p.pose.position.x << ", "
                  << p.pose.position.y << ", "
                  << p.pose.position.z << ", "
                  << roll << ", "
                  << pitch<< ", "
                  << yaw << ")\n";
    }
}

void save_arm_positions_to_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "arm_positions" << YAML::Value << YAML::BeginMap;
    for (const auto & [name, pose] : arm_positions) {
        auto & q = pose.pose.orientation;
        double roll, pitch, yaw;
        tf2::Quaternion quat= tf2::Quaternion(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3(tf2::Matrix3x3(quat)).getRPY(roll, pitch, yaw);
        out << YAML::Key << name << YAML::Value << YAML::BeginMap
            << YAML::Key << "x" << YAML::Value << pose.pose.position.x
            << YAML::Key << "y" << YAML::Value << pose.pose.position.y
            << YAML::Key << "z" << YAML::Value << pose.pose.position.z
            << YAML::Key << "roll" << YAML::Value <<roll
            << YAML::Key << "pitch" << YAML::Value <<pitch
            << YAML::Key << "yaw" << YAML::Value << yaw
            << YAML::EndMap;
    }
    out << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(arm_positions_filepath_);
    fout << out.c_str();
}

void clear_arm_positions_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "arm_positions" << YAML::Value << YAML::BeginMap
        << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(arm_positions_filepath_);
    fout << out.c_str();
}

void clear_arm_positions_map(){
    arm_positions.clear();
}

void clear_arm_positions(){
    clear_arm_positions_map();
    clear_arm_positions_yaml();
}

void add_arm_position(
    const std::string & name,
    double x, double y, double z, double roll, double pitch, double yaw,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    arm_positions[name] = make_arm_position(x, y, z, roll, pitch, yaw);
    save_arm_positions_to_yaml();
    cout << name << " -> " << problem->addInstance(plansys2::Instance{name, "arm_position"}) << endl;
}


void delete_arm_positions(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem)
{
    load_arm_positions_from_yaml();

    auto it = arm_positions.find(name);
    if (it == arm_positions.end()) {
        cout << "ERROR: Arm position '" << name << "' not found in arm_positions" << endl;
        return;
    }

    arm_positions.erase(it);
    save_arm_positions_to_yaml();

    cout << name << " -> " << problem->removeInstance(plansys2::Instance{name, "arm_position"}) << endl;
}


geometry_msgs::msg::PoseStamped get_arm_position(const std::string & name){
    load_arm_positions_from_yaml();
    auto it = arm_positions.find(name);
    if (it == arm_positions.end()) {
        throw std::runtime_error("ERROR: Arm position '" + name + "' not found in arm_positions");
    }
    return it->second;
}

std::string get_arm_position_str(const std::string & name){
    auto arm_position = get_arm_position(name);
    const auto & p = arm_position.pose.position;
    const auto & q = arm_position.pose.orientation;

    double roll, pitch, yaw;
    tf2::Quaternion quat= tf2::Quaternion(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tf2::Matrix3x3(quat)).getRPY(roll, pitch, yaw);

    std::ostringstream ss;
    ss << name << ":\n";
    ss << "        frame_id: " << arm_position.header.frame_id << "\n";
    ss << std::fixed << std::setprecision(3);
    ss << "        x: " << p.x << "\n";
    ss << "        y: " << p.y << "\n";
    ss << "        z: " << p.z << "\n";
    ss << "        roll: " << roll << "\n";
    ss << "        pitch: " << pitch << "\n";
    ss << "        yaw: " << yaw << "\n";
    return ss.str();
}