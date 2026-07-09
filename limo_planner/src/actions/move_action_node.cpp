#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


//miei
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


  void init_knowledge(){
    clear_all_map();
    load_waypoints_from_yaml(); 
    
    robot_name_ = get_arguments()[0];
    wp_from_name_ = get_arguments()[1];
    wp_dest_name_ = get_arguments()[2];  // The goal is in the 3rd argument of the action 
    RCLCPP_INFO(get_logger(), "Inizio navigazione verso [%s]", wp_dest_name_.c_str());

    goal_pos_ = get_waypoint(wp_dest_name_);
    //print_pose_stamped(goal_pos_);

    connection = get_connection(wp_from_name_, wp_dest_name_);
  }


  //! on_activate
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state){
    init_knowledge();
    send_feedback(0.0, "Move starting");

    problem_expert_client_ = std::make_shared<plansys2::ProblemExpertClient>();

    // --- CONTROLLO BATTERIA: se insufficiente, move fallisce subito e NON
    // viene nemmeno contattato Nav2. finish() è già stato chiamato dentro
    // check_battery_sufficient() nei casi di fallimento. ---
    battery_sufficient_ = check_battery_sufficient();
    if (!battery_sufficient_) {
      return ActionExecutorClient::on_activate(previous_state);
    }

    // --- Da qui in poi: codice di navigazione Nav2 invariato ---

    // 1. Creazione del client ROS2 (è una CLASSE)
    // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(shared_from_this(), "navigate_to_pose");


    bool is_action_server_ready = false;
    do {
      if (!rclcpp::ok()) {
        // Sostituisci RCLCPP_WARN con std::cout per evitare l'errore sul contesto invalido
        std::cout << "[MoveAction] Shutdown rilevato. Chiusura in corso..." << std::endl;
        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }

      RCLCPP_INFO(get_logger(), "In attesa del navigation action server...");
      is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
      
    } while (!is_action_server_ready);
    RCLCPP_INFO(get_logger(), "Navigation action server pronto");


    // nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    // Il target di navigazione è approach_wp (punto sul path a ridosso del
    // waypoint di destinazione), NON il waypoint di destinazione grezzo.
    navigation_goal_.pose = connection.approach_wp;

    // 2. CLIENT ROS2, STRUCT di configurazione delle opzioni di invio goal
    auto struct_callbacks_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

   
    struct_callbacks_goal_options.goal_response_callback = [this](std::shared_ptr<NavigationGoalHandle> goal_handle){
      if(goal_handle != NULL){
        RCLCPP_INFO(get_logger(), "GOAL accettato da NAV2");
      }else{
        RCLCPP_INFO(get_logger(), "GOAL RIFIUTATO da NAV2");
      }
    };


    struct_callbacks_goal_options.feedback_callback = [this]( NavigationGoalHandle::SharedPtr, NavigationFeedback feedback) {

      float distance_remaining = feedback->distance_remaining;
      float completed_distance_percent = 1.0 - (feedback->distance_remaining / connection.distance);
      send_feedback(completed_distance_percent, "Move in corso");
    };

    
    //! CHECK the docs for this:
    struct_callbacks_goal_options.result_callback = [this](const NavigationGoalHandle::WrappedResult & result) {
      switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
              finish(true, 1.0, "Move completata");
              break;
          case rclcpp_action::ResultCode::ABORTED:
              finish(false, 0.0, "Navigazione interrotta (aborted)");
              break;
          case rclcpp_action::ResultCode::CANCELED:
              finish(false, 0.0, "Navigazione annullata (cancelled)");
              break;
      }
      // result.result  - this is the Empty msg, useless
      // result.code    - THIS is what tells you what happened
      // result.goal_id - the UUID of the goal
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
    if(!battery_sufficient_){
      finish(false, 0.0, battery_not_sufficient_error_);
    }
  }


  //! SOLO FUNZIONI DI SUPPORTO QUI AVANTI:

  //! Ritorna energia necessaria per wp1 -> wp2
  std::optional<float> estimate_segment_energy(const Robot & rb, const std::string & from_wp, const std::string & to_wp){
    Connection cn;
    try {
      cn = get_connection(from_wp, to_wp);
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(get_logger(), "Connessione [%s -> %s] NON trovata nella knowledge base: %s",
                  from_wp.c_str(), to_wp.c_str(), e.what());
      return std::nullopt;
    }

    if (rb.max_robot_velocity <= 0.0f) {
      RCLCPP_WARN(get_logger(), "Il robot [%s] ha max_robot_velocity <= 0, impossibile stimare l'energia", robot_name_.c_str());
      return std::nullopt;
    }

    float costmap_factor = cn.costmap_estimate / 100.0;
    float travel_time = cn.distance / rb.max_robot_velocity;
    // float energy = rb.motor_power * travel_time * costmap_factor;
    float energy = rb.motor_power * travel_time; //!FIX THIS

    RCLCPP_INFO(get_logger(), "  segmento [%s -> %s]: distance=%.3f m, costmap_estimate=%.3f, "
                "costmap_factor=%.3f, energy=%.3f J",
                from_wp.c_str(), to_wp.c_str(), cn.distance, cn.costmap_estimate, costmap_factor, energy);

    return energy;
  }


  //! Ritorna il nome del waypoint della stazione, o std::nullopt se non trovata.
  std::optional<std::string> find_charging_station_waypoint(const std::string & from_wp){
    std::optional<std::string> closest_wp = std::nullopt;
    float closest_distance = std::numeric_limits<float>::max();

    for (const auto & pred : problem_expert_client_->getPredicates()) {
      if (pred.name != "charging_station_at" || pred.parameters.size() < 2) {
        continue;
      }
      std::string cs_wp = pred.parameters[1].name;

      float distance = 0.0f;  // 0 se il robot è già lì
      if (cs_wp != from_wp) {
        try {
          distance = get_connection(from_wp, cs_wp).distance;
        } catch (const std::runtime_error & e) {
          RCLCPP_WARN(get_logger(), "Stazione [%s] ignorata, nessuna connessione da [%s]: %s",
                      cs_wp.c_str(), from_wp.c_str(), e.what());
          continue;  // solo questa stazione viene scartata, non è un errore fatale
        }
      }

      if (distance < closest_distance) {
        closest_distance = distance;
        closest_wp = cs_wp;
      }
    }

    if (closest_wp.has_value()) {
      RCLCPP_INFO(get_logger(), "Stazione di ricarica più vicina a [%s]: [%s] (distanza=%.3f m)",
                  from_wp.c_str(), closest_wp.value().c_str(), closest_distance);
    } else {
      RCLCPP_WARN(get_logger(), "Nessuna stazione di ricarica raggiungibile da [%s]", from_wp.c_str());
    }

    return closest_wp;
  }


  // Rimuove il predicato (not_battery_low ?r) dalla knowledge base: segnala
  // al planner che il prossimo piano deve passare per la stazione di ricarica.
  void remove_not_battery_low_predicate(){
    bool removed = problem_expert_client_->removePredicate(
        plansys2::Predicate("(not_battery_low " + robot_name_ + ")"));
    if (removed) {
      RCLCPP_WARN(get_logger(), "Rimosso il predicato (not_battery_low %s): batteria troppo bassa per i segmenti pianificati",
                  robot_name_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "FALLITA la rimozione del predicato (not_battery_low %s)", robot_name_.c_str());
    }
  }


  //! Esegue il controllo batteria completo: wp_from -> wp_to -> charging_station.
  bool check_battery_sufficient(){
    Robot rb;
    try {
      rb = get_robot(robot_name_);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_logger(), "Robot [%s] non trovato: %s", robot_name_.c_str(), e.what());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Robot [%s]: current_battery=%.3f J", robot_name_.c_str(), rb.current_battery);

    auto charging_station_waypoint_opt = find_charging_station_waypoint(wp_dest_name_);
    if (!charging_station_waypoint_opt.has_value()) {
      //todo finish(false, 0.0, "No charging_station_at predicate found");
      battery_not_sufficient_error_ = "Nessun predicato charging_station_at trovato";
      return false;
    }
    std::string charging_station_waypoint = charging_station_waypoint_opt.value();

    // Tratto wp_from -> wp_to (quello che move sta per eseguire)
    auto energy_first_segment = estimate_segment_energy(rb, wp_from_name_, wp_dest_name_);
    if (!energy_first_segment.has_value()) {
      //todo finish(false, 0.0, "Missing connection data for wp_from -> wp_to, cannot check battery");
      battery_not_sufficient_error_ = "Dati di connessione mancanti per wp_from -> wp_to, impossibile controllare la batteria";
      return false;
    }

    // Tratto wp_to -> charging_station (per garantire che dopo move il robot
    // possa ancora raggiungere una stazione di ricarica)

    std::optional<float> energy_second_segment;
    if(wp_dest_name_ == charging_station_waypoint){
      energy_second_segment = 0.0;
    }else{
      energy_second_segment = estimate_segment_energy(rb, wp_dest_name_, charging_station_waypoint);
      if (!energy_second_segment.has_value()) {
        //todo finish(false, 0.0, "Missing connection data for wp_to -> charging_station, cannot check battery");
        battery_not_sufficient_error_ = "Dati di connessione mancanti per wp_to -> charging_station, impossibile controllare la batteria";
        return false;
      }
    }
    

    float total_energy_needed = energy_first_segment.value() + energy_second_segment.value();
    RCLCPP_INFO(get_logger(), "Energia totale necessaria: %.3f J (disponibile: %.3f J)",
                total_energy_needed, rb.current_battery);

    if (rb.current_battery < total_energy_needed) {
      // Batteria insufficiente: segnala al planner rimuovendo (not_battery_low r).
      // Il prossimo piano generato dovrà passare per la stazione di ricarica.
      remove_not_battery_low_predicate();
      //todo finish(false, 0.0, "Battery too low for wp_from -> wp_to -> charging_station");
      battery_not_sufficient_error_ = "Batteria troppo bassa per wp_from -> wp_to -> charging_station";
      return false;
    }

    return true;
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

  // --- NUOVO: stato per il controllo batteria ---
  std::string robot_name_;
  std::string wp_from_name_;
  std::string wp_dest_name_;
  Connection connection;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_client_;
  bool battery_sufficient_;
  std::string battery_not_sufficient_error_;

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
//todo verifica nome esatto campo plansys2_msgs::msg::Param (name vs value) per la tua versione di plansys2
//todo verifica firma esatta di ProblemExpertClient::removePredicate / getPredicates per Humble