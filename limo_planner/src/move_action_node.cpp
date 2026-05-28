#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction() : plansys2::ActionExecutorClient("move", 500ms){
    wp.header.frame_id = "map";
    wp.header.stamp = now();
    wp.pose.position.x = 0.0;
    wp.pose.position.y = -2.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = 1.0;


    using namespace std::placeholders;
    //subscriber a /amcl_pose, vuole una callback
    subscriber_to_amcl_position = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10, std::bind(&MoveAction::current_pos_callback, this, _1));
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    current_pos_ = msg->pose.pose;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state){
    send_feedback(0.0, "Move starting");

    // 1. Creazione del client ROS2 (è una CLASSE)
    // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(shared_from_this(), "navigate_to_pose");

    // bool is_action_server_ready = false;
    // do {
    //   RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

    //   is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    // } while (!is_action_server_ready);


    bool is_action_server_ready = false;
    do {
      // MODIFICA QUI: Se l'utente preme Ctrl+C, usciamo elegantemente senza loopare
      if (!rclcpp::ok()) {
        // Sostituisci RCLCPP_WARN con std::cout per evitare l'errore sul contesto invalido
        std::cout << "[MoveAction] Shutdown rilevato. Chiusura in corso..." << std::endl;
        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }

      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");
      is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
      
    } while (!is_action_server_ready);

    
    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    // auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    // RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    // geometry_msgs::msg::PoseStamped goal_pos_;
    goal_pos_ = wp;
    // nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    navigation_goal_.pose = goal_pos_;

    // 2. CLIENT ROS2, STRUCT di configurazione delle opzioni di invio goal
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    /*
    PLANSYS2:
    result_callback() e send_feedback():
    metodi di plansys2::ActionExecutorClient
    che pubblica su /actions_hub.
    */

    send_goal_options.goal_response_callback = [this](std::shared_ptr<NavigationGoalHandle> goal_handle) {
      RCLCPP_INFO(get_logger(), "PROVA send_goal_options.goal_response_callback");
    };

    send_goal_options.feedback_callback = [this]( NavigationGoalHandle::SharedPtr, NavigationFeedback feedback) {
        send_feedback(0.5, "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Move completed");
      };

    
    
    /*
    std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
    # con handle: puoi cancellare, monitorare, aspettare
    # send_goal_options(nav2_msgs::action::Action_X::Goal navigation_goal_, GOAL OPTION STRUCT OF CALLBACKS)
    */
    future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  void do_work(){
    RCLCPP_INFO(get_logger(), "PROVA do_work");
  }

  using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback = const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  // VA BENE QUI CLAUDE??
  geometry_msgs::msg::PoseStamped wp;


  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>> subscriber_to_amcl_position;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;

  //ACTION:
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> navigation_action_client_;
  std::shared_future<std::shared_ptr<NavigationGoalHandle>> future_navigation_goal_handle_;
  std::shared_ptr<NavigationGoalHandle> navigation_goal_handle_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  // node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->configure();   // invece di trigger_transition(TRANSITION_CONFIGURE)

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}


//todo leggiti: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html