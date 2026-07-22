#ifndef PARSER_HPP
#define PARSER_HPP

std::string trim(std::string s);

std::vector<std::string> split_string(const std::string & s, char delim);

std::string build_and_goal(const std::string & group);

bool ask_user_action(
    std::deque<std::string> & goal_queue,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert);

#endif 