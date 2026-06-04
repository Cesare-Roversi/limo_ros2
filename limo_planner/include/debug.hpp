#pragma once

void print_world_model(
    rclcpp::Node * node, //serve solo per node->get_logger()
    std::shared_ptr<plansys2::DomainExpertClient> domain,
    std::shared_ptr<plansys2::ProblemExpertClient> problem,
    bool print_structs = false);

