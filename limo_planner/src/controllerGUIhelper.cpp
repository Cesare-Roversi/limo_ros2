#include <plansys2_pddl_parser/Utils.h>
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include <stdexcept>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <deque>
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_msgs/msg/node.hpp"

#include "debug.hpp"
#include "world_data_utils.hpp"
#include "controller_functions.hpp"
#include "parser.hpp"
#include "arm_world_data_utils.hpp"


#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "visibility_control.h"
#include "limo_planner_interfaces/action/message.hpp"

using namespace std;

namespace limo_planner
{
class ControllerGUIhelper : public rclcpp::Node
{
public:
  using Message = limo_planner_interfaces::action::Message;
  using GoalHandleMessage = rclcpp_action::ServerGoalHandle<Message>;

  LIMO_PLANNER_CPP_PUBLIC
  explicit ControllerGUIhelper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("controllerGUIhelper", options), state_(GET_INPUT), old_state_(UNREACHABLE_LAST_STATE)
  {
    executing_print_counter_ = 0;
    executing_print_every_N_ = 4;
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Message>(
      this,
      "message",
      std::bind(&ControllerGUIhelper::handle_goal, this, _1, _2),
      std::bind(&ControllerGUIhelper::handle_cancel, this, _1),
      std::bind(&ControllerGUIhelper::handle_accepted, this, _1));
  }


private:
  rclcpp_action::Server<Message>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Message::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->operation);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandleMessage> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMessage> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ControllerGUIhelper::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMessage> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedbackmsg = std::make_shared<Message::Feedback>();
    auto & sequence = feedbackmsg->feedback;
    auto & statemsg = feedbackmsg->state;
    auto result = std::make_shared<Message::Result>();
    vector<string> waypoints;
    switch (goal->operation)
    {
    case 0:{//check_if_connected_list
        waypoints = split_string(goal->request, ',');
        for (auto & wp : waypoints) {
            wp = trim(wp);
        }
        check_if_connected_list(waypoints, goal_queue_, problem_expert_);
        break;
    }
    case 1:{//plan_patrol
        waypoints = split_string(goal->request, ',');
        for (auto & wp : waypoints) {
            wp = trim(wp);
        }
        plan_patrol("r1", waypoints, goal_queue_, problem_expert_);
        break;
    }
    case 2:{//insert_goal_manually
        vector<string> goal_groups = split_string(goal->request, '>');
        for (const auto & group : goal_groups) {
            string goal_str = build_and_goal(group);
            if (goal_str != "(and)") {
                goal_queue_.push_back(goal_str);
                cout << "Aggiunto goal alla coda: " << goal_str << endl;
            }
        }
        break;
    }
    case 3:{//shutdown
        shutdown_requested_ = true;
        break;
    }
    default:
        break;
    }
    valid_request = true;
    while (state_ == GET_INPUT) {
        statemsg = state_to_string(state_);
        goal_handle->publish_feedback(feedbackmsg);
        loop_rate.sleep();
    }
    while (state_ == PLANNING) {
        statemsg = state_to_string(state_);
        goal_handle->publish_feedback(feedbackmsg);
        loop_rate.sleep();
    }
    while (state_ == EXECUTING || !goal_queue_.empty()) {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        if(state_!=EXECUTING && statemsg != state_to_string(state_)){
            statemsg = state_to_string(state_);
            goal_handle->publish_feedback(feedbackmsg);
        }
        if (state_ == EXECUTING) {
            statemsg = state_to_string(state_);
            if (executing_print_counter_ % executing_print_every_N_ == 0) {
                auto feedback = executor_client_->getFeedBack();
                sequence.clear();
                sequence.reserve(feedback.action_execution_status.size());
                sequence.insert(sequence.end(), feedback.action_execution_status.begin(), feedback.action_execution_status.end());
                // Publish feedback
                goal_handle->publish_feedback(feedbackmsg);
            }
            executing_print_counter_++;
        }

        loop_rate.sleep();
    }
    if (success) {
        // Check if goal is done
        if (rclcpp::ok()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }

    } else {
        // Check if goal is done
        if (rclcpp::ok()) {
        result->success = false;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal not succeeded");
        }
    }

  }
  
public:
    typedef enum {GET_INPUT, PLANNING, EXECUTING, DEAD, UNREACHABLE_LAST_STATE} StateType;
    StateType state_;
    StateType old_state_;
    bool just_changed_state_;
    int executing_print_counter_;
    int executing_print_every_N_;
    std::deque<std::string> goal_queue_;
    bool shutdown_requested_;
    bool valid_request;
    bool success;

private:
    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    // rclcpp::TimerBase::SharedPtr timer_;

public:
    void change_state(StateType new_state){
        if(new_state >= GET_INPUT && new_state < UNREACHABLE_LAST_STATE){
            state_ = new_state;
        }else{
            throw std::runtime_error("ERROR: new_state doesn't exist");
        }
    }
    string state_to_string(StateType state){
        string state_str;
        switch (state){
            case GET_INPUT:
                state_str = "GET_INPUT";
                break;
            case PLANNING:
                state_str = "PLANNING";
                break;
            case EXECUTING:
                state_str = "EXECUTING";
                break;
            case DEAD:
                state_str = "DEAD";
                break;        
            default:
                throw std::runtime_error("ERROR: state doesn't exist");
        }
        return state_str;
    }

    void print_current_state(){
        switch (state_){
            case GET_INPUT:
                cout << endl << "======================== CURRENT STATE: GET_INPUT ========================" << endl;
                break;
            case PLANNING:
                cout << endl << "======================== CURRENT STATE: PLANNING ========================" << endl;
                break;
            case EXECUTING:
                cout << endl << "======================== CURRENT STATE: EXECUTING ========================" << endl;
                break;
            case DEAD:
                cout << endl << "======================== CURRENT STATE: DEAD ========================" << endl;
                break;        
            default:
                throw std::runtime_error("ERROR: state doesn't exist");
        }
    }

    void start_clients(){

        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        
    }

    void debug_init(){
        change_state(DEAD);
    }

    void init_knowledge(){
        cout << endl << endl << "INIT KNOWLEDGE" << endl;

        std::string pkg_share = ament_index_cpp::get_package_share_directory("limo_planner");
        waypoints_filepath_ = pkg_share + "/config/waypoints.yaml";
        objects_filepath_ = pkg_share + "/config/objects.yaml";
        robots_filepath_ = pkg_share + "/config/robots.yaml";
        arm_positions_filepath_ = pkg_share + "/config/arm_positions.yaml";
    //[-3.1, -0.48, 0.0]
        
        clear_all(); //knowledge //! ATTENTO ATTENO ATTENTO

        add_waypoint("wp0", 120.0, 32.0, 0.0, problem_expert_); //SPAWN
        add_waypoint("wp1", 125.2, 33.7, 0.0, problem_expert_); //di fronte allo spawn
        add_waypoint("wp2", 103.1,  10.6, 0.0, problem_expert_); //a metà corridoio centro sx
        add_waypoint("wp3", 0.3,  1.2, 0.0, problem_expert_);
        add_waypoint("wp4", 0.804443, 2.446670, -1.57,  problem_expert_);//SPAWN
        add_waypoint("wp5", -1.63, 1.96, 3.14,  problem_expert_);


        add_object("o", 1.0, 0.3, 0.3, 0.1, problem_expert_);
        problem_expert_->addPredicate(plansys2::Predicate("(object_at o wp4)"));

        add_robot("r1", 1.0, 1000.0, 1.0, 1.0, problem_expert_);
        problem_expert_->addPredicate(plansys2::Predicate("(arm_free r1)"));
        problem_expert_->addPredicate(plansys2::Predicate("(not_battery_low r1)"));
        problem_expert_->addPredicate(plansys2::Predicate("(doing_nothing r1)"));

        problem_expert_->addPredicate(plansys2::Predicate("(robot_at r1 wp4)"));

        problem_expert_->addInstance(plansys2::Instance("cs1", "charging_station"));
        problem_expert_->addPredicate(plansys2::Predicate("(charging_station_at cs1 wp3)"));


        add_arm_position("p0", -0.224, -0.1, 0.148, -1.57, 0.0, 0, problem_expert_);
        add_arm_position("p1", -0.20, 0.0, 0.16, 0.00, 1.00, 0.00, problem_expert_);
        problem_expert_->addPredicate(plansys2::Predicate("(object_at_arm_position o p0)"));

    }

    void step() {
        just_changed_state_ = !(old_state_ == state_);
        old_state_ = state_;
        
        if(just_changed_state_){
            print_current_state();
        }

        switch (state_) {
            case GET_INPUT:
            {   
                valid_request = false;
                while(!valid_request && !shutdown_requested_){
                }
                change_state(PLANNING);
                break;
            }

            case PLANNING: //? continua a riprovare finche non riesce a inizializzare
            {   
                problem_expert_->addPredicate(plansys2::Predicate("(doing_nothing r1)"));
                
                // problem_expert_->setGoal(plansys2::Goal("(and (object_at la_pimpa wp2))"));
                
                if(goal_queue_.empty()){
                    cout << "ERRORE: goal_queue_ è vuota nello stato PLANNING, NON dovrebbe succedere!" << endl;
                    change_state(GET_INPUT);
                    break;
                }

                print_goal_queue(goal_queue_);

                std::string first_goal = goal_queue_.front();
                bool goal_is_well_formed = problem_expert_->setGoal(plansys2::Goal(first_goal));
                if(!goal_is_well_formed){
                    cout << "Il Goal: " << first_goal << " è malformato" << endl;
                    change_state(DEAD);
                }

                auto domain = domain_expert_->getDomain();
                auto problem = problem_expert_->getProblem();
                auto plan = planner_client_->getPlan(domain, problem);

                if (!plan.has_value()) {
                    cout << "Impossibile trovare un piano che soddsifi: " << first_goal << endl;
                    change_state(DEAD);
                    break; //riferito allo switch (non controlla tutti i case)
                }

                if (executor_client_->start_plan_execution(plan.value())) {
                    cout << "Trovato un plan, transizione verso stato EXECUTING" << endl;
                    // state_ = EXECUTING;
                    change_state(EXECUTING);
                }
                break;
            }

            case EXECUTING:
            {
                success = false;

                if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                    if (executor_client_->getResult().value().success) {
                        success=true;
                        cout << "COMPLETATA esecuzione plan" << endl;
                        goal_queue_.pop_front();
                        if(goal_queue_.empty()){
                            change_state(GET_INPUT);
                        }else{
                            change_state(PLANNING);
                        }
                
                    } else {
                        cout << "FALLITO esecuzione del plan, transizione verso stato PLANNING" << endl;
                        change_state(PLANNING);
                    }
                }
                break;
            }

            case DEAD:
                break; //ho aggiunto lo stato DEAD xche deve restare così!
        }
    }
};  // class controllerGUIhelper

};  // namespace limo_planner




int main(int argc, char ** argv){
    cout << "HELLO_WORLD_1" << endl;

    rclcpp::init(argc, argv);

    auto node = std::make_shared<limo_planner::ControllerGUIhelper>();
    node->start_clients();
    node->init_knowledge();

    std::thread spin_thread([&node]() {
        rclcpp::spin(node->get_node_base_interface());
    });

    std::thread step_thread([&node]() {
        rclcpp::Rate rate(5);
        while (rclcpp::ok() && !node->shutdown_requested_) {
            node->step();
            rate.sleep();
        }
        rclcpp::shutdown();  // unblocks spin_thread's rclcpp::spin()
    });

    spin_thread.join();
    step_thread.join();

    cout << "END of controller.cpp" << endl;

    return 0;
}
