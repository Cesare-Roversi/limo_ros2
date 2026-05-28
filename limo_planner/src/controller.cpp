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
        
        // this->init_knowledge();

        cout << "IN" << endl;

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Controller::init_knowledge, this)
        );

        cout << "IN1" << endl;
    }

    void init_knowledge(){
        timer_->cancel(); //? schifo
        
        cout << endl << endl << "INIT KNOWLEDGE" << endl;
        cout << "r1 -> " << problem_expert_->addInstance(plansys2::Instance{"r1", "robot"}) << endl;
        cout << "wp1 -> " << problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"}) << endl;
        cout << "wp2 -> " << problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"}) << endl;

        cout << "(reachable wp1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(reachable wp1)")) << endl;
        cout << "(reachable wp2) -> " << problem_expert_->addPredicate(plansys2::Predicate("(reachable wp2)")) << endl;
		cout << "(robot_at r1 wp1) -> " << problem_expert_->addPredicate(plansys2::Predicate("(robot_at r1 wp1)")) << endl;
		cout << endl;

        // this->dump_plansys2_state();
        cout << "IN2" << endl;
    }

    void step() {
        cout << endl << endl << endl << "CURRENT STATE: " << state_ << endl;

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




    std::string goal_to_string(const plansys2_msgs::msg::Tree& tree, uint32_t node_id = 0) {
        if (tree.nodes.empty()) return "(none)";
        const auto& node = tree.nodes[node_id];
        
        if (node.node_type == plansys2_msgs::msg::Node::AND || 
            node.node_type == plansys2_msgs::msg::Node::OR) {
            std::string out = (node.node_type == plansys2_msgs::msg::Node::AND) ? "(and" : "(or";
            for (const auto& child_id : node.children) {
                out += " " + goal_to_string(tree, child_id);
            }
            return out + ")";
        } else if (node.node_type == plansys2_msgs::msg::Node::NOT) {
            return "(not " + goal_to_string(tree, node.children[0]) + ")";
        } else {
            // It's a predicate
            std::string out = "(" + node.name;
            for (const auto& p : node.parameters) {
                out += " " + p.name;
            }
            return out + ")";
        }
    }

    void dump_plansys2_state() {
        if (!domain_expert_ || !problem_expert_) {
            RCLCPP_ERROR(this->get_logger(), "Experts not initialized!");
            return;
        }

        std::stringstream ss;
        ss << "\n" << std::string(50, '=') << "\n";
        ss << "           PLANSYS2 CURRENT STATE\n";
        ss << std::string(50, '=') << "\n";

        // --- DOMAIN ---
        ss << "[DOMAIN TYPES]\n";
        for (const auto& type : domain_expert_->getTypes()) ss << "  - " << type << "\n";

        ss << "\n[DOMAIN PREDICATES]\n";
        for (const auto& pred : domain_expert_->getPredicates()) {
            ss << "  - (" << pred.name;
            for (const auto& p : pred.parameters) {
                ss << " " << p.name << ":" << p.type;
            }
            ss << ")\n";
        }

        ss << "\n[DOMAIN ACTIONS]\n";
        for (const auto& action_name : domain_expert_->getActions()) {
            auto action = domain_expert_->getAction(action_name);
            if (action) {
                ss << "  - (" << action_name;
                for (const auto& p : action->parameters) {
                    ss << " " << p.name << ":" << p.type;
                }
                ss << ")\n";
            }
        }

        // --- PROBLEM ---
        ss << "\n" << std::string(25, '-') << "\n";
        ss << "[PROBLEM INSTANCES]\n";
        for (const auto& inst : problem_expert_->getInstances()) {
            ss << "  - " << std::left << std::setw(15) << inst.name << " [" << inst.type << "]\n";
        }

        ss << "\n[PROBLEM PREDICATES (True Now)]\n";
        for (const auto& pred : problem_expert_->getPredicates()) {
            ss << "  - (" << pred.name;
            for (const auto& p : pred.parameters) {
                ss << " " << p.name; 
            }
            ss << ")\n";
        }

        ss << "\n[PROBLEM GOAL]\n";
        // Using our custom helper that won't throw compiler errors
        ss << "  " << goal_to_string(problem_expert_->getGoal()) << "\n";
        
        ss << std::string(50, '=');

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }


private:
    typedef enum {PLANNING, EXECUTING, FINISHED} StateType;
    StateType state_;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};





int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Controller>();
    node->init();

    // node->dump_plansys2_state
    
    cout << "PRE" << endl;
    rclcpp::sleep_for(std::chrono::seconds(10)); //!temporary
    cout << "POST" << endl;

    rclcpp::Rate rate(0.5); 
    while (rclcpp::ok()) {
        node->step();
        
        rate.sleep();
        rclcpp::spin_some(node->get_node_base_interface());
    }

    rclcpp::shutdown();
    return 0;
}