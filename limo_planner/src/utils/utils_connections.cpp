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
            map_connections[key] = Connection(distance, costmap_estimate);
        }
    }
}




void print_connections(){
    cout << "PRINT_CONNECTIONS" << endl;

    for (const auto& [key, cn] : map_connections) {
        std::cout << key << " -> ("
                  << cn.distance << ", "
                  << cn.costmap_estimate << ")\n";
    }
}

void save_connections_to_yaml(){
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "connections" << YAML::Value << YAML::BeginMap;
    for (const auto & [key, cn] : map_connections) {
        out << YAML::Key << key << YAML::Value << YAML::BeginMap
            << YAML::Key << "distance" << YAML::Value << cn.distance
            << YAML::Key << "costmap_estimate" << YAML::Value << cn.costmap_estimate
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
    bool affect_plansys2_kb,
    std::shared_ptr<plansys2::ProblemExpertClient> problem){

    std::string key = connection_key(wp1, wp2);
    map_connections[key] = Connection(distance, costmap_estimate);
    save_connections_to_yaml();
    cout << key << " -> connection saved (distance=" << distance
         << ", costmap_estimate=" << costmap_estimate << ")" << endl;

    if (affect_plansys2_kb) {
        if (!problem) {
            cout << "ERROR: affect_plansys2_kb=true but problem is nullptr, skipping predicate add" << endl;
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
        cout << "ERROR: Connection '" << key << "' not found in map_connections" << endl;
        return;
    }

    map_connections.erase(it);
    save_connections_to_yaml();

    cout << key << " -> connection removed" << endl;

    if (affect_plansys2_kb) {
        if (!problem) {
            cout << "ERROR: affect_plansys2_kb=true but problem is nullptr, skipping predicate remove" << endl;
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
        throw std::runtime_error("ERROR: Connection '" + key + "' not found in map_connections");
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
    float distance, float costmap_estimate){

    connections_filepath_ = filepath;
    load_connections_from_yaml();
    add_connection(wp1, wp2, distance, costmap_estimate);
}

//* __CONNECTIONS
