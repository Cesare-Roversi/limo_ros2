// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
  MoveAction()
  : plansys2::ActionExecutorClient("move", 500ms)
  {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = now();
    wp.pose.position.x = 0.0;
    wp.pose.position.y = -2.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = 1.0;
    waypoints_["wp1"] = wp;

    wp.pose.position.x = 1.8;
    wp.pose.position.y = 0.0;
    waypoints_["wp2"] = wp;

    wp.pose.position.x = 0.0;
    wp.pose.position.y = 2.0;
    waypoints_["wp3"] = wp;

    wp.pose.position.x = -0.5;
    wp.pose.position.y = -0.5;
    waypoints_["wp4"] = wp;

    wp.pose.position.x = -2.0;
    wp.pose.position.y = -0.4;
    waypoints_["wp_control"] = wp;

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&MoveAction::current_pos_callback, this, _1));
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    send_feedback(0.0, "Move starting");

    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");


    /*
    ESEMPIO LOGICA -> CPP
    (move lebron wp_control wp2)
    Mapping the array index looks like this:
    get_arguments()[0] -> "lebron" (The robot name)
    get_arguments()[1] -> "wp_control" (Where the robot is starting)
    get_arguments()[2] -> "wp2" (Where the robot needs to go)
    */


    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    /*
    PLANSYS2:
    è un metodo di plansys2::ActionExecutorClient
    che pubblica su /actions_hub.
    La mettiamo in una lamda per passarla a ??
    */
    send_goal_options.feedback_callback = [this]( NavigationGoalHandle::SharedPtr, NavigationFeedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Move completed");
      };
    
    
    /*
    * 1. COSA VIENE SETTATO:
      navigation_action_client_ salva:
      - send_goal_options= feedback + result
      - goal id (generato)
    * 2. COSA VIENE INVIATO:
      navigation_goal_ alla rete ros2
    * 3. COSA VIENE RESTITUITO:
      future_navigation_goal_handle_ che diventa un ClientGoalHandle valido.
      ClientGoalHandle ti permette di interagire con un particolare goal che hai mandato:
      metodi di ClientGoalHandle:
      get_status restituisce lo stato attuale dell’azione come esecuzione successo o cancellazione

      is_accepted indica se il server ha accettato o rifiutato il goal e ritorna un booleano

      get_goal_id restituisce l’identificatore univoco della missione

      async_get_result restituisce un future che permette di attendere il risultato finale dell’azione
    */
    future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  void do_work()
  {
  }

  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  /*
  In ROS 2, un file `.action` genera automaticamente tre strutture C++ standard
  Queste sono sempre presenti per ogni azione:
  NomeAzione::Goal -> input iniziale dell’azione (es. obiettivo)
  NomeAzione::Feedback -> aggiornamenti durante l’esecuzione
  NomeAzione::Result -> risultato finale dell’azione
  */

  using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback = const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;


  /*
  * std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>>
  equivale a questo:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
  !sostituisci !
  */
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;

  /*
  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>> future_navigation_goal_handle_;
  Azioni (Actions) compiti lunghi e complessi per esempio navigazione

  IN GENERALE:
  lato client o ricevitore rclcpp_action::ClientGoalHandler
  lato server o trasmettitore rclcpp_action::ServerGoalHandle

  Servizi (Services) richieste rapide domanda risposta per esempio accendi led
  lato client o ricevitore rclcpp::Client
  lato server o trasmettitore rclcpp::Service

  Messaggi (Topics) flusso continuo di dati unidirezionale per esempio laser odometria
  lato client o ricevitore rclcpp::Subscription
  lato server o trasmettitore rclcpp::Publisher

  */
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
