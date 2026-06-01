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

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_msgs/msg/node.hpp"

#include "debug.hpp"
#include "world_data_utils.hpp"

using namespace std;

class Controller : public rclcpp::Node
{
public:
    Controller()
    : rclcpp::Node("patrolling_controller"), state_(PLANNING)
    {
    }

    void init(){
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        
    }

    void init_knowledge(){
        
        cout << endl << endl << "INIT KNOWLEDGE" << endl;
        // cout << "r1 -> " << problem_expert_->addInstance(plansys2::Instance{"r1", "robot"}) << endl;
        // cout << "wp1 -> " << problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"}) << endl;
        // cout << "wp2 -> " << problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"}) << endl;

        add_waypoint("wp1", 125.2, 33.7, 0.0, problem_expert_); //di fronte allo spawn
        add_waypoint("wp2", 103.1,  10.6, 0.0, problem_expert_); //a metà corridoio centro sx
        add_waypoint("wp3", 166.3,  47.4, 0.0, problem_expert_); //medio-alto a sx
        add_waypoint("wp4", 167.8, 16.4, 0.0,  problem_expert_); //medio-alto a dx
        add_waypoint("wp5", 18.1, 25.2, 0.0,  problem_expert_); //basoo in centro

        cout << "r1 -> " << problem_expert_->addInstance(plansys2::Instance{"r1", "robot"}) << endl;

        cout << "(reachable wp1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(reachable wp1)")) << endl;
        cout << "(reachable wp2) -> " << problem_expert_->addPredicate(plansys2::Predicate("(reachable wp2)")) << endl;
		cout << "(robot_at r1 wp1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(robot_at r1 wp1)")) << endl;
		cout << endl;

        print_world_model(this, this->domain_expert_, this->problem_expert_);
    }

    void step() {
        cout << endl << "CURRENT STATE: " << state_ << endl;

        switch (state_) {
            case PLANNING: //? continua a riprovare finche non riesce a inizializzare
            {
                problem_expert_->setGoal(plansys2::Goal("(and (robot_at r1 wp2))"));
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
                    state_ = EXECUTING;
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
                        state_ = FINISHED; 
                    } else {
                        cout << "FALLITO esecuzione del plan, transizione verso stato PLANNING" << endl;
                        state_ = PLANNING; 
                    }
                }
                break;
            }

            case FINISHED:
                break;
        }
    }


private:
    typedef enum {PLANNING, EXECUTING, FINISHED} StateType;
    StateType state_;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    // rclcpp::TimerBase::SharedPtr timer_;
};





int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    // cout << waypoints["wp1"] << endl;

    auto node = std::make_shared<Controller>();
    node->init();
    rclcpp::sleep_for(std::chrono::seconds(3)); //!temporary
    node->init_knowledge();

    rclcpp::Rate rate(5); //! era 0.5
    while (rclcpp::ok()) {
        node->step();
        
        rate.sleep();
        rclcpp::spin_some(node->get_node_base_interface());
    }


    rclcpp::shutdown();
    return 0;
}