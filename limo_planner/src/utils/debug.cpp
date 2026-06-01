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
    rclcpp::Node * node, //serve solo per node->get_logger()
    std::shared_ptr<plansys2::DomainExpertClient> domain,
    std::shared_ptr<plansys2::ProblemExpertClient> problem)
{
    std::stringstream ss;
    ss << "\n" << std::string(50, '=') << "\n";
    ss << "           PLANSYS2 CURRENT STATE\n";
    ss << std::string(50, '=') << "\n";

    ss << "[DOMAIN TYPES]\n";
    for (const auto & t : domain->getTypes()) ss << "  - " << t << "\n";

    ss << "\n[DOMAIN PREDICATES]\n";
    for (const auto & pred : domain->getPredicates()) {
        ss << "  - (" << pred.name;
        for (const auto & p : pred.parameters) ss << " " << p.name << ":" << p.type;
        ss << ")\n";
    }

    ss << "\n[DOMAIN ACTIONS]\n";
    for (const auto & name : domain->getActions()) {
        auto action = domain->getAction(name);
        if (!action) continue;
        ss << "  - (" << name;
        for (const auto & p : action->parameters) ss << " " << p.name << ":" << p.type;
        ss << ")\n";
    }

    ss << "\n" << std::string(25, '-') << "\n";
    ss << "[INSTANCES]\n";
    for (const auto & inst : problem->getInstances())
        ss << "  - " << std::left << std::setw(15) << inst.name << " [" << inst.type << "]\n";

    ss << "\n[PREDICATES TRUE]\n";
    for (const auto & pred : problem->getPredicates()) {
        ss << "  - (" << pred.name;
        for (const auto & p : pred.parameters) ss << " " << p.name;
        ss << ")\n";
    }

    ss << "\n[GOAL]\n";
    ss << "  " << goal_to_string(problem->getGoal()) << "\n";
    ss << std::string(50, '=');

    RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
}