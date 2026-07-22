#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


//miei
#include "world_data_utils.hpp"
#include "debug.hpp"

using namespace std::chrono_literals;
using namespace std;

class CheckDistanceAction : public plansys2::ActionExecutorClient
{
public:
  CheckDistanceAction() : plansys2::ActionExecutorClient("check_distance", 500ms){
    // NB: qui non serve is_initial_distance_set/initial_distance come in MoveAction,
    // perché questa azione NON naviga: si limita a CALCOLARE il path tra 2 waypoint.
  }

  void print_pose_stamped(const geometry_msgs::msg::PoseStamped& msg){
    std::cout << endl << "WAYPOINT:" << std::endl;
    std::cout << "header.frame_id: " << msg.header.frame_id << std::endl;
    std::cout << "header.stamp: " << msg.header.stamp.sec << "."
              << msg.header.stamp.nanosec << std::endl;

    std::cout << "position: (" << msg.pose.position.x << ", "
                              << msg.pose.position.y << ", "
                              << msg.pose.position.z << ")" << std::endl;

    std::cout << "orientation: (" << msg.pose.orientation.x << ", "
                                << msg.pose.orientation.y << ", "
                                << msg.pose.orientation.z << ", "
                                << msg.pose.orientation.w << ")" << endl << endl;
  }


  void init_knowledge(){
    std::string pkg_share = ament_index_cpp::get_package_share_directory("limo_planner");
    waypoints_filepath_ = pkg_share + "/config/waypoints.yaml";
    objects_filepath_ = pkg_share + "/config/objects.yaml";
    robots_filepath_ = pkg_share + "/config/robots.yaml";

    clear_all_map();
    load_waypoints_from_yaml();

    // I 2 waypoint arrivano come parametri dell'azione PDDL: (?wp1 ?wp2)
    auto wp1_name = get_arguments()[0];
    auto wp2_name = get_arguments()[1];
    RCLCPP_INFO(get_logger(), "Checking distance between [%s] and [%s]", wp1_name.c_str(), wp2_name.c_str());

    start_pos_ = get_waypoint(wp1_name);
    goal_pos_ = get_waypoint(wp2_name);

    print_pose_stamped(start_pos_);
    print_pose_stamped(goal_pos_);
  }


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state){
    init_knowledge();
    send_feedback(0.0, "Check_distance starting");

    // 1. Creazione del client ROS2 per l'azione ComputePathToPose
    compute_path_action_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(shared_from_this(), "compute_path_to_pose");

    bool is_action_server_ready = false;
    do {
      if (!rclcpp::ok()) {
        std::cout << "[CheckDistanceAction] Shutdown rilevato. Chiusura in corso..." << std::endl;
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }

      RCLCPP_INFO(get_logger(), "Waiting for compute_path_to_pose action server...");
      is_action_server_ready = compute_path_action_client_->wait_for_action_server(std::chrono::seconds(5));

    } while (!is_action_server_ready);
    RCLCPP_INFO(get_logger(), "compute_path_to_pose action server ready");


    // 2. Costruzione del GOAL: qui, a differenza di MoveAction, impostiamo
    // ESPLICITAMENTE anche lo start (non usiamo la posizione attuale del robot),
    // perché vogliamo il path teorico tra wp1 e wp2, non dal robot al goal.
    compute_path_goal_.start = start_pos_;
    compute_path_goal_.goal = goal_pos_;
    compute_path_goal_.planner_id = "GridBased";   // stesso planner_id usato di default da Nav2
    compute_path_goal_.use_start = true;           // true => usa "start" sopra invece della pos. attuale del robot

    auto struct_callbacks_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();

    struct_callbacks_goal_options.goal_response_callback = [this](std::shared_ptr<ComputePathGoalHandle> goal_handle){
      if(goal_handle != NULL){
        RCLCPP_INFO(get_logger(), "GOAL (compute_path_to_pose) was accepted by NAV2");
      }else{
        RCLCPP_INFO(get_logger(), "GOAL (compute_path_to_pose) was REFUSED by NAV2");
        // Se il goal viene rifiutato, il result_callback NON verrà mai chiamato:
        // dobbiamo far fallire noi l'azione qui, altrimenti resta bloccata per sempre.
        finish(false, 0.0, "compute_path_to_pose goal refused");
      }
    };

    // ComputePathToPose non ha un feedback definito (nessun campo), quindi non serve
    // registrare una feedback_callback qui (a differenza di MoveAction).

    struct_callbacks_goal_options.result_callback = [this](const ComputePathGoalHandle::WrappedResult & result) {
      switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
          {
            // --- QUI salviamo il contenuto del result in variabili membro ---
            // result.result è un ComputePathToPose::Result, che (in Humble) contiene:
            //   nav_msgs/Path path                     -> la sequenza di pose calcolata
            //   builtin_interfaces/Duration planning_time -> tempo impiegato dal planner
            computed_path_ = result.result->path;
            planning_time_ = result.result->planning_time;

            // Un path vuoto (nessuna pose) significa in pratica "nessun path trovato":
            // in tal caso trattiamo l'azione come fallita anche se il ResultCode è SUCCEEDED.
            if (computed_path_.poses.empty()) {
              RCLCPP_WARN(get_logger(), "compute_path_to_pose returned an EMPTY path");
              finish(false, 0.0, "No valid path found (empty path)");
              break;
            }

            RCLCPP_INFO(get_logger(), "Path computed: %zu poses, planning_time: %d.%09d s",
                        computed_path_.poses.size(),
                        planning_time_.sec,
                        planning_time_.nanosec);

            // Azione PDDL riuscita: l'effetto (connected ?wp1 ?wp2) verrà applicato da PlanSys2
            finish(true, 1.0, "Check_distance completed: waypoints are connected");
            break;
          }
          case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_WARN(get_logger(), "compute_path_to_pose ABORTED -> waypoints NOT connected");
              finish(false, 0.0, "compute_path_to_pose aborted");
              break;
          case rclcpp_action::ResultCode::CANCELED:
              finish(false, 0.0, "compute_path_to_pose cancelled");
              break;
      }
    };

    future_compute_path_goal_handle_ = compute_path_action_client_->async_send_goal(compute_path_goal_, struct_callbacks_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  void do_work(){
    RCLCPP_INFO(get_logger(), "PROVA do_work (check_distance)");
  }

  using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>;

  // WAYPOINTS coinvolti nel check
  geometry_msgs::msg::PoseStamped start_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;

  // ComputePathToPose ACTION:
  nav2_msgs::action::ComputePathToPose::Goal compute_path_goal_;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>> compute_path_action_client_;
  std::shared_future<std::shared_ptr<ComputePathGoalHandle>> future_compute_path_goal_handle_;

  // --- Variabili dove salviamo il RESULT dell'azione ---
  nav_msgs::msg::Path computed_path_;               // il path calcolato (sequenza di PoseStamped)
  builtin_interfaces::msg::Duration planning_time_;  // tempo impiegato dal planner per calcolarlo

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CheckDistanceAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "check_distance"));
  node->configure();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}


//todo ComputePathToPose (Humble) result: nav_msgs/Path path, builtin_interfaces/Duration planning_time
//     (NIENTE error_code/error_msg in Humble: quelli sono stati aggiunti in versioni successive di Nav2)
//todo ACTION ROS2: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
//todo PLANSYS2 RILEGGITI: https://plansys2.github.io/tutorials/docs/simple_example.html