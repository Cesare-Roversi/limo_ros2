#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveWithObjectAction : public plansys2::ActionExecutorClient
{
public:
  MoveWithObjectAction()
  : plansys2::ActionExecutorClient("move_with_object", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) { //sto lavorando
      progress_ += 0.02;
      send_feedback(progress_, "DOING: moveWithObject"); //! IMPORTANTE

    } else { //ho finito

      finish(true, 1.0, "ENDED: moveWithObject"); //! IMPORTANTE

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
  auto node = std::make_shared<MoveWithObjectAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move_with_object"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
