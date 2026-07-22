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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;

extern string arm_positions_filepath_;

extern unordered_map<std::string, geometry_msgs::msg::PoseStamped> arm_positions;


//*ARM POSITIONS
geometry_msgs::msg::PoseStamped make_arm_position(float x, float y, float z, float roll, float pitch, float yaw);
void load_arm_positions_from_yaml();
void save_arm_positions_to_yaml();
void clear_arm_positions_yaml();
void clear_arm_positions_map();
void clear_arm_positions();
void print_arm_poisitions(); //!solo DEBUG

void add_arm_position(
    const std::string & name,
    double x, double y, double z, double roll, double pitch, double yaw,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);
void delete_arm_position(

    const std::string & name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem);

geometry_msgs::msg::PoseStamped get_arm_position(const std::string & name);
std::string get_arm_position_str(const std::string & name);
