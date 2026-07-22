#ifndef GET_USER_INPUT_HPP
#define GET_USER_INPUT_HPP

bool ask_user_action(
	std::deque<std::string> & goal_queue,
	std::shared_ptr<plansys2::ProblemExpertClient> problem_expert, bool& shutdown_requested_);

#endif 