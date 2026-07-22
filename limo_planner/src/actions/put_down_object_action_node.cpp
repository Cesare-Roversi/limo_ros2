#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "arm_world_data_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using namespace std;

class PutDownObjectAction : public plansys2::ActionExecutorClient
{
public:

  PutDownObjectAction() : plansys2::ActionExecutorClient("put_down_object", 500ms){}

  void init_knowledge(){
    start_time_ = now();
    RCLCPP_INFO(get_logger(), "PutDownObject: avvio rilascio oggetto per %.1f secondi", ACTION_DURATION);

    auto const moveit_node = std::make_shared<rclcpp::Node>(
        "put_down_object",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = new MoveGroupInterface(moveit_node,"gripper_group");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state){
    init_knowledge();
    send_feedback(0.0, "Sto posando l'oggetto");
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  moveit::planning_interface::MoveGroupInterface *move_group_interface;
  
  void do_work(){
    double elapsed = (now() - start_time_).seconds();

    if (elapsed >= ACTION_DURATION) {
      finish(true, 1.0, "Put down completed");
      return;
    }

    string pkg_share = ament_index_cpp::get_package_share_directory("limo_planner");
    arm_positions_filepath_ = pkg_share+"/config/arm_positions.yaml";
    clear_arm_positions_map();
    load_arm_positions_from_yaml();

    const auto & args = get_arguments();
    if (args.size()<2){
        RCLCPP_ERROR(get_logger(), "put_down_object: gli argomenti richiesti sono insufficienti. Sono attesi due argomenti: (robot, oggetto) ");
        finish(false, 0.0, "Put down object couldn't start");
        return;
    }
    move_group_interface->setNamedTarget("open");
        moveit::planning_interface::MoveGroupInterface *move_group_interface_lambda=move_group_interface;
    auto const [success, plan] = [move_group_interface_lambda]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok= static_cast <bool> (move_group_interface_lambda->plan(msg));
        return std::make_pair(ok, msg);
    }();
    if(success){
        move_group_interface->execute(plan);
    } else {
        RCLCPP_ERROR(get_logger(), "put_down_object: pianificazione fallita ");
        finish(false, 0.0, "Put down object: planning failed");
        return;
    }

    float progress = static_cast<float>(elapsed / ACTION_DURATION);
    std::string msg = "Sto posando l'oggetto " + std::to_string(static_cast<int>(elapsed)) + "s";
    send_feedback(progress, msg);
  }

  static constexpr double ACTION_DURATION = 5.0;
  rclcpp::Time start_time_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PutDownObjectAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "put_down_object"));
  node->configure();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}