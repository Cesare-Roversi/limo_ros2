#pragma once

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

//tutti i tipi plansys2 sono un plansys2_msgs::msg::Node
//il goal è un vettore di Node[], devi parsarlo!
// # Node types
// uint8 AND = 1
// uint8 OR = 2
// uint8 NOT = 3
// uint8 ACTION = 4
// uint8 PREDICATE = 5
// uint8 FUNCTION = 6
// uint8 EXPRESSION = 7
// uint8 FUNCTION_MODIFIER = 8
// uint8 NUMBER = 9


std::string goal_to_string(const plansys2_msgs::msg::Tree & tree, uint32_t node_id = 0)
{
    if (tree.nodes.empty()) return "(none)";
    const auto & node = tree.nodes[node_id];

    if (node.node_type == plansys2_msgs::msg::Node::AND ||
        node.node_type == plansys2_msgs::msg::Node::OR)
    {
        std::string out = (node.node_type == plansys2_msgs::msg::Node::AND) ? "(and" : "(or";
        for (const auto & child_id : node.children)
            out += " " + goal_to_string(tree, child_id);
        return out + ")";
    }

    if (node.node_type == plansys2_msgs::msg::Node::NOT)
        return "(not " + goal_to_string(tree, node.children[0]) + ")";

    std::string out = "(" + node.name;
    for (const auto & p : node.parameters) out += " " + p.name;
    return out + ")";
}



void print_world_model(
    rclcpp::Node * node,
    std::shared_ptr<plansys2::DomainExpertClient> domain,
    std::shared_ptr<plansys2::ProblemExpertClient> problem)
{
    std::stringstream ss;
    ss << "\n" << std::string(50, '=') << "\n";
    ss << "           PLANSYS2 CURRENT STATE\n";
    ss << std::string(50, '=') << "\n";

    std::vector<std::string> types = domain->getTypes();
    std::vector<plansys2::Predicate> domain_predicates = domain->getPredicates();
    std::vector<plansys2::Instance> instances = problem->getInstances();
    std::vector<plansys2::Predicate> active_predicates = problem->getPredicates();
    // plansys2::Goal goal_tree = problem->getPredicates(); //???

    ss << "[DOMAIN TYPES]\n";
    for (const std::string & t : types)
        ss << "  - " << t << "\n";

    ss << "\n[DOMAIN PREDICATES]\n";
    for (const plansys2::Predicate & pred : domain_predicates)
        ss << "  - " << pred.name << "\n";

    ss << "\n" << std::string(25, '-') << "\n";
    ss << "[INSTANCES]\n";
    for (const plansys2::Instance & inst : instances)
        ss << "  - " << std::left << std::setw(15) << inst.name << " [" << inst.type << "]\n";

    ss << "\n[PREDICATES TRUE]\n";
    for (const plansys2::Predicate & pred : active_predicates) {
        ss << "  - (" << pred.name;
        for (const plansys2_msgs::msg::Param & p : pred.parameters)
            ss << " " << p.name;
        ss << ")\n";
    }

    ss << "\n[GOAL]\n";
    ss << "  " << goal_to_string(problem->getGoal()) << "\n";
    ss << std::string(50, '=');

    RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
}


// void print_world_model(
//     rclcpp::Node * node, //serve solo per node->get_logger()
//     std::shared_ptr<plansys2::DomainExpertClient> domain,
//     std::shared_ptr<plansys2::ProblemExpertClient> problem)
// {
//     std::stringstream ss;
//     ss << "\n" << std::string(50, '=') << "\n";
//     ss << "           PLANSYS2 CURRENT STATE\n";
//     ss << std::string(50, '=') << "\n";

//     ss << "[DOMAIN TYPES]\n";
//     for (const auto & t : domain->getTypes()) ss << "  - " << t << "\n";

//     ss << "\n[DOMAIN PREDICATES]\n";
//     for (const auto & pred : domain->getPredicates()) {
//         ss << "  - " << pred.name << "\n";
//     }

//     ss << "\n" << std::string(25, '-') << "\n";
//     ss << "[INSTANCES]\n";
//     for (const auto & inst : problem->getInstances())
//         ss << "  - " << std::left << std::setw(15) << inst.name << " [" << inst.type << "]\n";

//     ss << "\n[PREDICATES TRUE]\n";
//     for (const auto & pred : problem->getPredicates()) {
//         ss << "  - (" << pred.name;
//         for (const auto & p : pred.parameters) ss << " " << p.name;
//         ss << ")\n";
//     }

//     ss << "\n[GOAL]\n";
//     ss << "  " << goal_to_string(problem->getGoal()) << "\n";
//     ss << std::string(50, '=');

//     RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
// }



//! VERSIONE VECCHIA: CORRETTA MA IN PRATICA PLANSYS2 NON RESTITUISCE TUTTO, NELLA NUOVO AGGIUNGO ANCHE IL CONTENUTO DELLE MAP
// void print_world_model(
//     rclcpp::Node * node, //serve solo per node->get_logger()
//     std::shared_ptr<plansys2::DomainExpertClient> domain,
//     std::shared_ptr<plansys2::ProblemExpertClient> problem)
// {
//     std::stringstream ss;
//     ss << "\n" << std::string(50, '=') << "\n";
//     ss << "           PLANSYS2 CURRENT STATE\n";
//     ss << std::string(50, '=') << "\n";

//     ss << "[DOMAIN TYPES]\n";
//     for (const auto & t : domain->getTypes()) ss << "  - " << t << "\n";

//     ss << "\n[DOMAIN PREDICATES]\n";
//     for (const auto & pred : domain->getPredicates()) {
//         // ss << "QUI: " << pred.parameters.size(); //! VETTORE PARAMTERI VUOTO ANCHE SE ESISTIONO NEL DOMINIO!, è UN BUG?
//         ss << "  - (" << pred.name;
//         for (const auto & p : pred.parameters) ss << " " << p.name << ":" << p.type;
//         ss << ")\n";
//     }

//     ss << "\n[DOMAIN ACTIONS]\n";
//     // ss << "QUI: " << domain->getActions().size();
//     for (const auto & name : domain->getActions()) { //! VETTORE AZIONI VUOTO ANCHE SE ESISTIONO NEL DOMINIO!, è UN BUG?
//         auto action = domain->getAction(name);
//         if (!action) continue;
//         ss << "  - (" << name;
//         for (const auto & p : action->parameters) ss << " " << p.name << ":" << p.type;
//         ss << ")\n";
//     }

//     ss << "\n" << std::string(25, '-') << "\n";
//     ss << "[INSTANCES]\n";
//     for (const auto & inst : problem->getInstances())
//         ss << "  - " << std::left << std::setw(15) << inst.name << " [" << inst.type << "]\n";

//     ss << "\n[PREDICATES TRUE]\n";
//     for (const auto & pred : problem->getPredicates()) {
//         ss << "  - (" << pred.name;
//         for (const auto & p : pred.parameters) ss << " " << p.name;
//         ss << ")\n";
//     }

//     ss << "\n[GOAL]\n";
//     ss << "  " << goal_to_string(problem->getGoal()) << "\n";
//     ss << std::string(50, '=');

//     RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
// }