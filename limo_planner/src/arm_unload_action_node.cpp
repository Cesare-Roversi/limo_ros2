#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class UnloadAction : public plansys2::ActionExecutorClient
{
public:
  UnloadAction()
  : plansys2::ActionExecutorClient("arm_unload", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) { //sto lavorando
      progress_ += 0.02;
      send_feedback(progress_, "DOING: unload"); //! IMPORTANTE

    } else { //ho finito

      finish(true, 1.0, "ENDED: unload"); //! IMPORTANTE

      progress_ = 0.0;
      // std::cout << std::endl;
    }

    // std::cout << "\r\e[K" << std::flush;
    // std::cout << "Moving ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
    // std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnloadAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "arm_unload"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
