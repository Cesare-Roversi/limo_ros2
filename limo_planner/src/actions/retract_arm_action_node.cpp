#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std;

class RetractArmAction : public plansys2::ActionExecutorClient
{
public:
  RetractArmAction() : plansys2::ActionExecutorClient("retract_arm", 500ms){}

  void init_knowledge(){
    start_time_ = now();
    RCLCPP_INFO(get_logger(), "RetractArm: avvio retrazione braccio per %.1f secondi", ACTION_DURATION);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state){
    init_knowledge();
    send_feedback(0.0, "Sto retraendo il braccio");
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  void do_work(){
    double elapsed = (now() - start_time_).seconds();

    if (elapsed >= ACTION_DURATION) {
      finish(true, 1.0, "Retract arm completed");
      return;
    }

    float progress = static_cast<float>(elapsed / ACTION_DURATION);
    std::string msg = "Sto retraendo il braccio " + std::to_string(static_cast<int>(elapsed)) + "s";
    send_feedback(progress, msg);
  }

  static constexpr double ACTION_DURATION = 5.0;
  rclcpp::Time start_time_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RetractArmAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "retract_arm"));
  node->configure();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}