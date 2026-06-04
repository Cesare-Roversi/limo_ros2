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

#include "world_data_utils.hpp"
#include "debug.hpp"

using namespace std::chrono_literals;
using namespace std;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction() : plansys2::ActionExecutorClient("move", 500ms){
    initial_distance = -1; //x debug
    is_initial_distance_set = false;

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

    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    goal_pos_ = map_waypoints[wp_to_navigate]; //! ERROR IS HERE, PASSA UN WP VUOTO PERCHé????
    // nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    navigation_goal_.pose = goal_pos_;

    // 2. CLIENT ROS2, STRUCT di configurazione delle opzioni di invio goal
    auto struct_callbacks_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    /*
    PLANSYS2:
    result_callback() e send_feedback():
    metodi di plansys2::ActionExecutorClient
    che pubblica su /actions_hub.
    */

    struct_callbacks_goal_options.goal_response_callback = [this](std::shared_ptr<NavigationGoalHandle> goal_handle){
      if(goal_handle != NULL){
        RCLCPP_INFO(get_logger(), "GOAL was accepted by NAV2");
      }else{
        RCLCPP_INFO(get_logger(), "GOAL was REFUSED by NAV2");
      }
    };

    struct_callbacks_goal_options.feedback_callback = [this]( NavigationGoalHandle::SharedPtr, NavigationFeedback feedback) {

        float distance_remaining = feedback->distance_remaining;

        if(distance_remaining > 0.3 || is_initial_distance_set){
          if(!is_initial_distance_set){ //0.3 è precauzione perchè qualche volta restituisce valori bassi senza senso all'inizio;
            initial_distance = feedback->distance_remaining;
            is_initial_distance_set = true;
          }
          
          float completed_distance_percent = 1.0 - (feedback->distance_remaining / initial_distance);
          send_feedback(completed_distance_percent, "Move running");
          // RCLCPP_INFO(get_logger(), "distance_remaining: %.3f m (%.3f /)", feedback->distance_remaining, completed_distance_percent);
        }else{
          RCLCPP_INFO(get_logger(), "DEBUG: feedback->distance_remaining under limit: %.3f m", distance_remaining);
        }
      };
    
    //! CHECK the docs for this:
    struct_callbacks_goal_options.result_callback = [this](const NavigationGoalHandle::WrappedResult & result) {
      switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
              finish(true, 1.0, "Move completed");
              break;
          case rclcpp_action::ResultCode::ABORTED:
              // add_waypoint("fail_01", )
              finish(false, 0.0, "Navigation aborted");
              break;
          case rclcpp_action::ResultCode::CANCELED:
              finish(false, 0.0, "Navigation cancelled");
              break;
      }
      // result.result  → this is the Empty msg, useless
      // result.code    → THIS is what tells you what happened
      // result.goal_id → the UUID of the goal
  };

    
    
    /*
    std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
    # con handle: puoi cancellare, monitorare, aspettare
    # struct_callbacks_goal_options(nav2_msgs::action::Action_X::Goal navigation_goal_, GOAL OPTION STRUCT OF CALLBACKS)
    */
    future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, struct_callbacks_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  void do_work(){
    RCLCPP_INFO(get_logger(), "PROVA do_work");
  }

  using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback = const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  // TEST:
  geometry_msgs::msg::PoseStamped wp;

  //amcl_pose TOPIC:
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>> subscriber_to_amcl_position;
  geometry_msgs::msg::Pose current_pos_;

  //NavigateToPose ACTION:
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> navigation_action_client_;
  std::shared_future<std::shared_ptr<NavigationGoalHandle>> future_navigation_goal_handle_;
  std::shared_ptr<NavigationGoalHandle> navigation_goal_handle_;

  //OTHER:
  float initial_distance;
  bool is_initial_distance_set;

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


//todo ACTION ROS2: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
//todo PLANSYS2 RILEGGITI: https://plansys2.github.io/tutorials/docs/simple_example.html
//todo ROS2 SPIN: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html