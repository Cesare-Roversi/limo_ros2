#include "world_data_utils.hpp"

/*
load_*_from_yaml //salva tutta la mappa nel yaml
save_*_to_yaml   //carica tutto il yaml nella mappa
add_*            //aggiunge * alla mappa -> salva tutta la mappa nel yaml
delete_*         //elimina * dalla mappa -> salva tutta la mappa nel yaml
get_*            //restituisce (in base al nome) una struct o classe C++
*/




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
            float current_battery = rb.second["current_battery"].as<float>();
            map_robots[name] = Robot(battery_voltage, battery_mah, motor_power, max_robot_velocity, current_battery);
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
                  << rb.max_robot_velocity << ", "
                  << rb.current_battery << ")\n";
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
            << YAML::Key << "current_battery" << YAML::Value << rb.current_battery
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

    Robot rb(battery_voltage, battery_mah, motor_power, max_robot_velocity);
    rb.current_battery = rb.battery_joules();  // nuovo robot -> batteria piena
    map_robots[name] = rb;
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

// Aggiorna solo current_battery di un robot già esistente (es. dopo un move
// che consuma energia, o dopo un charge che la ripristina) e persiste su YAML.
void set_robot_battery(const std::string & name, float new_current_battery){
    load_robots_from_yaml();

    auto it = map_robots.find(name);
    if (it == map_robots.end()) {
        throw std::runtime_error("ERROR: Robot '" + name + "' not found in map_robots");
    }

    it->second.current_battery = new_current_battery;
    save_robots_to_yaml();
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
    ss << "        current_battery: " << rb.current_battery << "\n";
    return ss.str();
}
//*__ROBOTS
