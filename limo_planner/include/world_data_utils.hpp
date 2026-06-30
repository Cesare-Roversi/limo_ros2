#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <plansys2_pddl_parser/Utils.h>
#include <memory>
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include <fstream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "world_data_structs.hpp"

using namespace std;

extern std::string waypoints_filepath_;
extern std::string objects_filepath_;
extern std::string robots_filepath_;

extern std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> map_waypoints;
extern std::unordered_map<std::string, Object> map_objects;
extern std::unordered_map<std::string, Robot> map_robots;


//*WAYPOINTS
geometry_msgs::msg::PoseStamped make_waypoint(float x, float y, float yaw);
void load_waypoints_from_yaml();
void save_waypoints_to_yaml();
void clear_waypoints_yaml();
void clear_waypoints_map();
void clear_waypoints();
void print_waypoints(); //!solo DEBUG

void add_waypoint(
    const std::string & name,
    double x, double y, double yaw,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);
void delete_waypoint(

    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);

geometry_msgs::msg::PoseStamped get_waypoint(const std::string & name);
std::string get_waypoint_str(const std::string & name);

//*__WAYPOINTS


//*OBJECTS
void load_objects_from_yaml();
void print_objects();
void save_objects_to_yaml();
void clear_objects_yaml();
void clear_objects_map();
void clear_objects();

void add_object(
    const std::string & name,
    float height, float max_width, float min_width, float weight,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);

void delete_object(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);

Object get_object(const std::string & name);

std::string get_object_str(const std::string & name);

//*__OBJECTS


//*ROBOTS:
void load_robots_from_yaml();
void print_robots();
void save_robots_to_yaml();
void clear_robots_yaml();
void clear_robots_map();
void clear_robots();

void add_robot(
    const std::string & name,
    float battery_voltage, float battery_mah, float motor_power, float max_robot_velocity,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);

void delete_robot(
    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);

Robot get_robot(const std::string & name);

std::string get_robot_str(const std::string & name);

//*__ROBOTS


std::string get_robot_str(const std::string & name);


//*RAGGRUPPAMENTO:
std::string get_instance_str(const std::string& type, const std::string& name);

void clear_all_yaml();
void clear_all_map();
void clear_all();