#include <plansys2_pddl_parser/Utils.h>
#include <memory>
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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
#include "get_user_input.hpp"



using namespace std;

class Controller : public rclcpp::Node{

public:
    typedef enum {GET_INPUT, PLANNING, EXECUTING, DEAD, UNREACHABLE_LAST_STATE} StateType;
    StateType state_;
    StateType old_state_;
    bool just_changed_state_;
    int executing_print_counter_;
    int executing_print_every_N_;
    std::deque<std::string> goal_queue_;
    bool shutdown_requested_;


private:
    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    // rclcpp::TimerBase::SharedPtr timer_;

public:
    Controller()
    : rclcpp::Node("patrolling_controller"), state_(GET_INPUT), old_state_(UNREACHABLE_LAST_STATE)
    {
        executing_print_counter_ = 0;
        executing_print_every_N_ = 4;
    }

    void change_state(StateType new_state){
        if(new_state >= GET_INPUT && new_state < UNREACHABLE_LAST_STATE){
            state_ = new_state;
        }else{
            throw std::runtime_error("ERROR: new_state doesn't exist");
        }
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


        clear_all(); //knowledge //! ATTENTO ATTENO ATTENTO

        add_waypoint("wp0", 120.0, 32.0, 0.0, problem_expert_); //SPAWN
        add_waypoint("wp1", 125.2, 33.7, 0.0, problem_expert_); //di fronte allo spawn
        add_waypoint("wp2", 103.1,  10.6, 0.0, problem_expert_); //a metà corridoio centro sx
        add_waypoint("wp3", 166.3,  47.4, 0.0, problem_expert_); //medio-alto a sx
        add_waypoint("wp4", 167.8, 16.4, 0.0,  problem_expert_); //medio-alto a dx
        add_waypoint("wp5", 18.1, 25.2, 0.0,  problem_expert_); //basso in centro

        // cout << "(connected wp0 wp1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(connected wp0 wp1)")) << endl;
        // cout << "(connected wp1 wp0) -> " << problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp0)")) << endl;
        // cout << "(connected wp1 wp2) -> " << problem_expert_->addPredicate(plansys2::Predicate("(connected wp0 wp1)")) << endl;
        // cout << "(connected wp2 wp1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp0)")) << endl;

        add_object("la_pimpa", 2.0, 3.0, 3.0, 1.0, problem_expert_);
        cout << "(object_at la_pimpa wp1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(object_at la_pimpa wp1)")) << endl;

        add_object("l_armando", 8.0, 3.0, 3.0, 1.0, problem_expert_);

        add_robot("r1", 1.0, 1000.0, 1.0, 1.0, problem_expert_);
        // cout << "(arm_retracted r1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(arm_retracted r1)")) << endl;
        cout << "(arm_free r1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(arm_free r1)")) << endl;
        cout << "(not_battery_low r1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(not_battery_low r1)")) << endl;
        cout << "(doing_nothing r1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(doing_nothing r1)"));

        cout << "(robot_at r1 wp0) -> " << problem_expert_->addPredicate(plansys2::Predicate("(robot_at r1 wp0)")) << endl;

        cout << "cs1 instance -> " << problem_expert_->addInstance(plansys2::Instance("cs1", "charging_station")) << endl;
        cout << "(charging_station_at cs1 wp1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(charging_station_at cs1 wp1)")) << endl;
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
                print_world_model(this, this->domain_expert_, this->problem_expert_, true);

                bool valid_request = false;
                while(!valid_request && !shutdown_requested_){
                    valid_request = ask_user_action(goal_queue_, problem_expert_, shutdown_requested_);
                }

                // goal_queue_.push_back("(and (arm_retracted r1))");
                // goal_queue_.push_back("(and (connected wp0 wp1))");

                change_state(PLANNING);
                break;
            }

            case PLANNING: //? continua a riprovare finche non riesce a inizializzare
            {   
                cout << "(doing_nothing r1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(doing_nothing r1)")) << endl;
                
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
                auto feedback = executor_client_->getFeedBack();

                if (executing_print_counter_ % executing_print_every_N_ == 0) {
                    for (const auto & action_feedback : feedback.action_execution_status) {
                        cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]";
                    }
                    cout << endl;
                }
                executing_print_counter_++;

                if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                    if (executor_client_->getResult().value().success) {
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

};




int main(int argc, char ** argv){
    cout << "HELLO_WORLD_1" << endl;

    rclcpp::init(argc, argv);

    auto node = std::make_shared<Controller>();
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



// int main(int argc, char ** argv){

//     cout << "HELLO_WORLD_1" << endl;

//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<Controller>();
//     node->start_clients();
//     //rclcpp::sleep_for(std::chrono::seconds(3));
//     node->init_knowledge();

//     std::thread spin_thread([&node]() {
//         rclcpp::spin(node->get_node_base_interface());
//     });

//     std::thread step_thread([&node]() {
//         rclcpp::Rate rate(5);
//         while (rclcpp::ok()) {
//             node->step();
//             rate.sleep();
//         }
//     });

//     spin_thread.join();
//     step_thread.join();

//     cout << "END of controller.cpp" << endl;

//     rclcpp::shutdown();
//     return 0;
// }