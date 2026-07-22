#include "world_data_utils.hpp"

/*
load_*_from_yaml //salva tutta la mappa nel yaml
save_*_to_yaml   //carica tutto il yaml nella mappa
add_*            //aggiunge * alla mappa -> salva tutta la mappa nel yaml
delete_*         //elimina * dalla mappa -> salva tutta la mappa nel yaml
get_*            //restituisce (in base al nome) una struct o classe C++
*/



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



