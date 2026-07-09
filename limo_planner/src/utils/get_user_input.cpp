#include <string>
#include <vector>
#include <cstdio>
#include <iostream>  // std::cout, std::endl
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "controller_functions.hpp"
#include "parser.hpp"
#include "get_user_input.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;

bool ask_user_action(
	std::deque<std::string> & goal_queue,
	std::shared_ptr<plansys2::ProblemExpertClient> problem_expert, bool& shutdown_requested_)
{
	while (true) {
		cout << "Cosa vuoi fare?" << endl;
		cout << "  1) Verifica connessione waypoint (check_if_connected_list)" << endl;
		cout << "  2) Pianifica pattugliamento (plan_patrol)" << endl;
		cout << "  3) Inserisci goal manualmente" << endl;
		cout << "  4) Spegni il nodo" << endl;
		cout << "Scrivi 1, 2, 3, 4 oppure 'exit' per uscire" << endl;
		cout << "> ";

		std::string choice;
		std::getline(std::cin, choice);

		if (choice == "exit") {
			return false;  // esce del tutto, niente da pianificare
		}

		if (choice == "1") {
			// ---- Opzione 1: check_if_connected_list ----
			cout << "Inserisci i waypoint separati da virgola (es: wp1, wp2, wp3)" << endl;
			cout << "> ";

			std::string wp_line;
			std::getline(std::cin, wp_line);

			if (wp_line == "exit") {
				continue;  // torna al menu, non fa nulla
			}

			std::vector<std::string> waypoints = split_string(wp_line, ',');
			for (auto & wp : waypoints) {
				wp = trim(wp);
			}

			return check_if_connected_list(waypoints, goal_queue, problem_expert);
			
		} else if (choice == "2") {
			// ---- Opzione 2: plan_patrol ----
			cout << "Nome del robot" << endl;
			cout << "> ";

			std::string robot_name;
			std::getline(std::cin, robot_name);

			if (robot_name == "exit") {
				continue;  // torna al menu
			}

			cout << "Inserisci i waypoint separati da virgola (es: wp1, wp2, wp3)" << endl;
			cout << "> ";

			std::string wp_line;
			std::getline(std::cin, wp_line);

			if (wp_line == "exit") {
				continue;  // torna al menu
			}

			std::vector<std::string> waypoints = split_string(wp_line, ',');
			for (auto & wp : waypoints) {
				wp = trim(wp);
			}

			return plan_patrol(robot_name, waypoints, goal_queue, problem_expert); 
		} else if (choice == "3") {
			// ---- Opzione 3: inserimento goal manuale ----
			cout << "Inserisci i goal (virgola = and, '>' = goal successivi in coda)" << endl;
			cout << "> ";

			std::string input_line;
			std::getline(std::cin, input_line);

			if (input_line == "exit") {
				continue;  // torna al menu
			}

			std::vector<std::string> goal_groups = split_string(input_line, '>');
			for (const auto & group : goal_groups) {
				std::string goal_str = build_and_goal(group);
				if (goal_str != "(and)") {
					goal_queue.push_back(goal_str);
					cout << "Aggiunto goal alla coda: " << goal_str << endl;
				}
			}

			return true;
		} else if (choice == "4") {
			shutdown_requested_ = true;
			return false;
		} else {
			cout << "Scelta non valida, riprova." << endl;
			// resta nel ciclo, ripropone il menu
		}
	}
}