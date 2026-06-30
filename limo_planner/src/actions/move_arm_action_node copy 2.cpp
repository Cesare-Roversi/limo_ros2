#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std;

class ChargeAction : public plansys2::ActionExecutorClient
{
public:
  ChargeAction() : plansys2::ActionExecutorClient("charge", 500ms){}

  void init_knowledge(){
    start_time_ = now();
    RCLCPP_INFO(get_logger(), "Charge: avvio carica per %.1f secondi", CHARGE_DURATION);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state){
    init_knowledge();
    send_feedback(0.0, "Mi sto caricando");
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  void do_work(){
    double elapsed = (now() - start_time_).seconds();

    if (elapsed >= CHARGE_DURATION) {
      finish(true, 1.0, "Charge completed");
      return;
    }

    float progress = static_cast<float>(elapsed / CHARGE_DURATION);
    std::string msg = "Mi sto caricando " + std::to_string(static_cast<int>(elapsed)) + "s";
    send_feedback(progress, msg);
  }

  static constexpr double CHARGE_DURATION = 10.0;
  rclcpp::Time start_time_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChargeAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "charge"));
  node->configure();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}