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
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_msgs/msg/node.hpp"

#include "debug.hpp"
#include "world_data_utils.hpp"
#include "arm_world_data_utils.hpp"

using namespace std;

class Controller : public rclcpp::Node{

public:
    typedef enum {PLANNING, EXECUTING, FINISHED, DEAD, UNREACHABLE_LAST_STATE} StateType;
    StateType state_;
    StateType old_state_;
    bool just_changed_state_;

private:
    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    // rclcpp::TimerBase::SharedPtr timer_;

public:
    Controller()
    : rclcpp::Node("patrolling_controller"), state_(PLANNING), old_state_(UNREACHABLE_LAST_STATE)
    {
    }

    void change_state(StateType new_state){
        if(new_state >= PLANNING && new_state <= UNREACHABLE_LAST_STATE){
            state_ = new_state;
        }else{
            throw std::runtime_error("ERROR: new_state doesn't exist");
        }
    }

    void print_current_state(){
        switch (state_){
            case PLANNING:
                cout << endl << "======================== CURRENT STATE: PLANNING ========================" << endl;
                break;
            case EXECUTING:
                cout << endl << "======================== CURRENT STATE: EXECUTING ========================" << endl;
                break;
            case FINISHED:
                cout << endl << "======================== CURRENT STATE: FINISHED ========================" << endl;
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

        clear_all(); //knowledge

        add_waypoint("wp0", 120.0, 32.0, 0.0, problem_expert_); //SPAWN
        add_object("o", 1.0, 0.3, 0.3, 0.1, problem_expert_);
        cout << "(object_at o wp0) -> " << problem_expert_->addPredicate(plansys2::Predicate("(object_at o wp0)")) << endl;
        add_robot("r1", 1.0, 1.0, 1.0, 1.0, problem_expert_);
        cout << "(arm_free r1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(arm_free r1)")) << endl;
        cout << "(not_battery_low r1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(not_battery_low r1)")) << endl;
        cout << "(doing_nothing r1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(doing_nothing r1)"));
        cout << "(robot_at r1 wp0) -> " << problem_expert_->addPredicate(plansys2::Predicate("(robot_at r1 wp0)")) << endl;
        cout << "cs1 instance -> " << problem_expert_->addInstance(plansys2::Instance("cs1", "charging_station")) << endl;
        cout << "(charging_station_at cs1 wp0) -> " << problem_expert_->addPredicate(plansys2::Predicate("(charging_station_at cs1 wp0)")) << endl;
        add_arm_position("p0", 0.20, 0.0, 0.15, 0.0, 1.57, 0.0, problem_expert_);
        add_arm_position("p1", -0.20, 0.0, 0.16, 0.00, 1.00, 0.00, problem_expert_);
        cout << "(object_at_arm_position o p0) -> " << problem_expert_->addPredicate(plansys2::Predicate("(object_at_arm_position o p0)")) << endl;


        print_world_model(this, this->domain_expert_, this->problem_expert_, true);
        //change_state(DEAD);
    }

    void step() {
        just_changed_state_ = !(old_state_ == state_);
        old_state_ = state_;
        
        if(just_changed_state_){
            print_current_state();
        }

        switch (state_) {
            case PLANNING: //? continua a riprovare finche non riesce a inizializzare
            {

                problem_expert_->setGoal(plansys2::Goal("(and (object_at_arm_position o p1))"));

                cout << "THE GOAL IS: " << parser::pddl::toString(problem_expert_->getGoal()) << endl; //lo imposta giusto

                auto domain = domain_expert_->getDomain();
                auto problem = problem_expert_->getProblem();
                auto plan = planner_client_->getPlan(domain, problem);

                if (!plan.has_value()) {
                    cout << "Could not find plan to reach the goal: " << parser::pddl::toString(problem_expert_->getGoal()) << endl;
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
                
                for (const auto & action_feedback : feedback.action_execution_status) {
                    cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]";
                }
                cout << endl;

                if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                    if (executor_client_->getResult().value().success) {
                        cout << "COMPLETATA esecuzione plan, transizione verso stato FINISHED" << endl;
                        // state_ = FINISHED; 
                        change_state(FINISHED);
                    } else {
                        cout << "FALLITO esecuzione del plan, transizione verso stato PLANNING" << endl;
                        // state_ = PLANNING; 
                        change_state(PLANNING);
                    }
                }
                break;
            }

            case FINISHED:
                break;

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
    rclcpp::sleep_for(std::chrono::seconds(3));
    node->init_knowledge();

    std::thread spin_thread([&node]() {
        rclcpp::spin(node->get_node_base_interface());
    });

    std::thread step_thread([&node]() {
        rclcpp::Rate rate(5);
        while (rclcpp::ok()) {
            node->step();
            rate.sleep();
        }
    });

    spin_thread.join();
    step_thread.join();

    rclcpp::shutdown();
    return 0;
}