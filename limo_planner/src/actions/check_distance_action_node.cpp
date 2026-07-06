#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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

// Numero di pose campionate lungo il path per stimare la vicinanza a ostacoli
// statici sulla global_costmap (arbitrario, come da richiesta).
static constexpr int kCostmapSamples = 10;

class CheckDistanceAction : public plansys2::ActionExecutorClient
{
public:
  CheckDistanceAction() : plansys2::ActionExecutorClient("check_distance", 500ms){
    // NB: qui non serve is_initial_distance_set/initial_distance come in MoveAction,
    // perché questa azione NON naviga: si limita a CALCOLARE il path tra 2 waypoint.
  }


  void init_knowledge(){

    clear_all_map();
    load_waypoints_from_yaml();
    load_connections_from_yaml();

    // I 2 waypoint arrivano come parametri dell'azione PDDL: (?wp1 ?wp2)
    wp1_name_ = get_arguments()[0];
    wp2_name_ = get_arguments()[1];

    RCLCPP_INFO(get_logger(), "Checking distance between [%s] and [%s]", wp1_name_.c_str(), wp2_name_.c_str());

    start_pos_ = get_waypoint(wp1_name_);
    goal_pos_ = get_waypoint(wp2_name_);

    print_pose_stamped(start_pos_);
    print_pose_stamped(goal_pos_);
  }


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state){
    init_knowledge();
    send_feedback(0.0, "Check_distance starting");

    // Sottoscrizione alla global_costmap: ci serve per stimare la vicinanza a
    // ostacoli statici campionando N punti lungo il path calcolato.
    // QoS transient_local perché costmap è tipicamente pubblicata cosi da Nav2.
    rclcpp::QoS costmap_qos(1);
    costmap_qos.transient_local();
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", costmap_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        latest_costmap_ = msg;
      });

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

            // --- Calcolo distanza e stima costmap, poi salvataggio in connections.yaml ---
            double path_distance = compute_path_length(computed_path_);
            double costmap_estimate = sample_costmap_along_path(computed_path_, kCostmapSamples);

            RCLCPP_INFO(get_logger(), "distance=%.3f m, costmap_estimate=%.3f (avg cost over %d samples)",
                        path_distance, costmap_estimate, kCostmapSamples);

            // affect_plansys2_kb=false: l'effetto (connected ?wp1 ?wp2) viene già
            // applicato automaticamente da PlanSys2 tramite finish(true, ...) qui
            // sotto, dato che è l'effetto dichiarato dell'azione PDDL. Aggiungerlo
            // anche qui a mano sarebbe ridondante (e servirebbe un
            // problem_expert_client_ che questo nodo non ha).
            add_connection(wp1_name_, wp2_name_, path_distance, costmap_estimate,
                            false, nullptr);

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

  // Somma delle distanze euclidee 2D tra pose consecutive del path.
  double compute_path_length(const nav_msgs::msg::Path & path){
    double length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & prev = path.poses[i - 1].pose.position;
      const auto & curr = path.poses[i].pose.position;
      double dx = curr.x - prev.x;
      double dy = curr.y - prev.y;
      length += std::sqrt(dx * dx + dy * dy);
    }
    return length;
  }

  // Campiona N pose equispaziate lungo il path e ne legge il costo sulla
  // global_costmap (valori 0-100, -1/unknown, 100=occupato). Ritorna il costo
  // medio sui campioni validi. Se la costmap non è ancora arrivata, ritorna -1.0
  // e logga un warning (non blocca l'azione: è solo una stima indicativa).
  double sample_costmap_along_path(const nav_msgs::msg::Path & path, int n_samples){
    if (!latest_costmap_) {
      RCLCPP_WARN(get_logger(), "No costmap received yet on /global_costmap/costmap, skipping estimate");
      return -1.0;
    }
    if (path.poses.empty() || n_samples <= 0) {
      return -1.0;
    }

    const auto & info = latest_costmap_->info;
    double sum_cost = 0.0;
    int valid_samples = 0;

    int n = std::min(static_cast<int>(path.poses.size()), n_samples);
    for (int i = 0; i < n; ++i) {
      // indice campionato uniformemente lungo il path
      size_t idx = (path.poses.size() == 1)
        ? 0
        : static_cast<size_t>(i * (path.poses.size() - 1) / std::max(1, n - 1));

      const auto & p = path.poses[idx].pose.position;

      int cell_x = static_cast<int>((p.x - info.origin.position.x) / info.resolution);
      int cell_y = static_cast<int>((p.y - info.origin.position.y) / info.resolution);

      if (cell_x < 0 || cell_y < 0 ||
          cell_x >= static_cast<int>(info.width) || cell_y >= static_cast<int>(info.height)) {
        continue;  // fuori dai bordi della costmap, salta il campione
      }

      int8_t cost = latest_costmap_->data[cell_y * info.width + cell_x];
      if (cost < 0) {
        continue;  // -1 = unknown, non lo conto nella media
      }

      sum_cost += cost;
      ++valid_samples;
    }

    return (valid_samples > 0) ? (sum_cost / valid_samples) : -1.0;
  }

  using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>;

  // WAYPOINTS coinvolti nel check
  std::string wp1_name_;
  std::string wp2_name_;
  geometry_msgs::msg::PoseStamped start_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;

  // ComputePathToPose ACTION:
  nav2_msgs::action::ComputePathToPose::Goal compute_path_goal_;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>> compute_path_action_client_;
  std::shared_future<std::shared_ptr<ComputePathGoalHandle>> future_compute_path_goal_handle_;

  // --- Variabili dove salviamo il RESULT dell'azione ---
  nav_msgs::msg::Path computed_path_;               // il path calcolato (sequenza di PoseStamped)
  builtin_interfaces::msg::Duration planning_time_;  // tempo impiegato dal planner per calcolarlo

  // Costmap globale, usata solo per il sampling: non persistita, solo in RAM.
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
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