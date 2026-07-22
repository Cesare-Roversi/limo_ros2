#include "world_data_utils.hpp"
#include <filesystem>

/*
load_*_from_yaml //salva tutta la mappa nel yaml
save_*_to_yaml   //carica tutto il yaml nella mappa
add_*            //aggiunge * alla mappa -> salva tutta la mappa nel yaml
delete_*         //elimina * dalla mappa -> salva tutta la mappa nel yaml
get_*            //restituisce (in base al nome) una struct o classe C++
*/


//* CONNECTIONS
// Chiave composita "wp1_wp2" (stringa) per identificare la coppia di waypoint.
inline std::string connection_key(const std::string & wp1, const std::string & wp2){
    return wp1 + "_" + wp2;
}

void load_connections_from_yaml(){
    map_connections.clear();
    YAML::Node config = YAML::LoadFile(connections_filepath_);
    if (config["connections"]) {
        for (const auto & cn : config["connections"]) {
            std::string key = cn.first.as<std::string>();
            float distance = cn.second["distance"].as<float>();
            float costmap_estimate = cn.second["costmap_estimate"].as<float>();

            double x = cn.second["approach_wp"]["x"].as<double>();
            double y = cn.second["approach_wp"]["y"].as<double>();
            double yaw = cn.second["approach_wp"]["yaw"].as<double>();

            geometry_msgs::msg::PoseStamped approach_wp;
            approach_wp.header.frame_id = "map";
            approach_wp.pose.position.x = x;
            approach_wp.pose.position.y = y;
            approach_wp.pose.position.z = 0.0;
            approach_wp.pose.orientation.x = 0.0;
            approach_wp.pose.orientation.y = 0.0;
            approach_wp.pose.orientation.z = std::sin(yaw / 2.0);
            approach_wp.pose.orientation.w = std::cos(yaw / 2.0);

            map_connections[key] = Connection(distance, costmap_estimate, approach_wp);
        }
    }
}



void print_connections(){
    cout << "PRINT_CONNECTIONS" << endl;

    for (const auto& [key, cn] : map_connections) {
        std::cout << key << " -> ("
                  << cn.distance << ", "
                  << cn.costmap_estimate << ", approach_wp=("
                  << cn.approach_wp.pose.position.x << ", "
                  << cn.approach_wp.pose.position.y << "))\n";
    }
}

void save_connections_to_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "connections" << YAML::Value << YAML::BeginMap;
    for (const auto & [key, cn] : map_connections) {
        double yaw = std::atan2(
            2.0 * (cn.approach_wp.pose.orientation.w * cn.approach_wp.pose.orientation.z),
            1.0 - 2.0 * (cn.approach_wp.pose.orientation.z * cn.approach_wp.pose.orientation.z));

        out << YAML::Key << key << YAML::Value << YAML::BeginMap
            << YAML::Key << "distance" << YAML::Value << cn.distance
            << YAML::Key << "costmap_estimate" << YAML::Value << cn.costmap_estimate
            << YAML::Key << "approach_wp" << YAML::Value << YAML::BeginMap
                << YAML::Key << "x" << YAML::Value << cn.approach_wp.pose.position.x
                << YAML::Key << "y" << YAML::Value << cn.approach_wp.pose.position.y
                << YAML::Key << "yaw" << YAML::Value << yaw
            << YAML::EndMap
            << YAML::EndMap;
    }
    out << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(connections_filepath_);
    fout << out.c_str();
}

void clear_connections_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "connections" << YAML::Value << YAML::BeginMap
        << YAML::EndMap << YAML::EndMap;

    std::ofstream fout(connections_filepath_);
    fout << out.c_str();
}

void clear_connections_map(){
    map_connections.clear();
}

void clear_connections(){
    clear_connections_map();
    clear_connections_yaml();
}

void add_connection(
    const std::string & wp1, const std::string & wp2,
    float distance, float costmap_estimate,
    const geometry_msgs::msg::PoseStamped & approach_wp,
    bool affect_plansys2_kb,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    std::string key = connection_key(wp1, wp2);
    map_connections[key] = Connection(distance, costmap_estimate, approach_wp);
    save_connections_to_yaml();
    cout << key << " -> connection saved (distance=" << distance
         << ", costmap_estimate=" << costmap_estimate
         << ", approach_wp=(" << approach_wp.pose.position.x << ", "
         << approach_wp.pose.position.y << "))" << endl;

    if (affect_plansys2_kb) {
        if (!problem) {
            cout << "ERROR[add_connection]: affect_plansys2_kb=true but problem is nullptr, skipping predicate add" << endl;
            return;
        }
        std::string pred_str = "(connected " + wp1 + " " + wp2 + ")";
        cout << pred_str << " -> " << problem->addPredicate(plansys2::Predicate(pred_str)) << endl;
    }
}

void delete_connection(
    const std::string & wp1, const std::string & wp2,
    bool affect_plansys2_kb,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    load_connections_from_yaml();

    std::string key = connection_key(wp1, wp2);
    auto it = map_connections.find(key);
    if (it == map_connections.end()) {
        cout << "ERROR[delete_connection]: Connection '" << key << "' not found in map_connections" << endl;
        return;
    }

    map_connections.erase(it);
    save_connections_to_yaml();

    cout << key << " -> connection removed" << endl;

    if (affect_plansys2_kb) {
        if (!problem) {
            cout << "ERROR[delete_connection]: affect_plansys2_kb=true but problem is nullptr, skipping predicate remove" << endl;
            return;
        }
        std::string pred_str = "(connected " + wp1 + " " + wp2 + ")";
        cout << pred_str << " -> " << problem->removePredicate(plansys2::Predicate(pred_str)) << endl;
    }
}

Connection get_connection(const std::string & wp1, const std::string & wp2){
    load_connections_from_yaml();
    std::string key = connection_key(wp1, wp2);
    auto it = map_connections.find(key);
    if (it == map_connections.end()) {
        throw std::runtime_error("ERROR [get_connection]: Connection '" + key + "' not found in map_connections");
    }
    return it->second;
}

std::string get_connection_str(const std::string & wp1, const std::string & wp2){
    auto cn = get_connection(wp1, wp2);
    std::string key = connection_key(wp1, wp2);

    std::ostringstream ss;
    ss << key << ":\n";
    ss << std::fixed << std::setprecision(3);
    ss << "        distance: " << cn.distance << "\n";
    ss << "        costmap_estimate: " << cn.costmap_estimate << "\n";
    return ss.str();
}

// Salva/aggiorna una connessione ricaricando prima lo stato da disco, cosi
// istanze diverse (es. altri ActionExecutorClient) non si sovrascrivono a
// vicenda lo YAML — stesso motivo per cui delete_robot fa load prima di erase.
void save_connection_to_yaml(
    const std::string & filepath,
    const std::string & wp1, const std::string & wp2,
    float distance, float costmap_estimate,
    const geometry_msgs::msg::PoseStamped & approach_wp){

    connections_filepath_ = filepath;
    load_connections_from_yaml();
    add_connection(wp1, wp2, distance, costmap_estimate, approach_wp);
}

//* __CONNECTIONS