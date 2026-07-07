

#include <deque>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iostream>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "world_data_utils.hpp"
#include "controller_functions.hpp"

using namespace std;

// ============================================================
// check_if_connected_list
// ------------------------------------------------------------
// Itera su tutte le coppie ordinate (wp_i, wp_j) con i != j
// della lista fornita. Per ogni coppia il cui predicato
// (connected wp_i wp_j) non è ancora istanziato nella problem
// expert, viene inserito un goal corrispondente nella coda.
// ============================================================
void check_if_connected_list(
    const std::vector<std::string> & waypoints,
    std::deque<std::string> & goal_queue,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert)
{
    for (size_t i = 0; i < waypoints.size(); ++i) {
        for (size_t j = 0; j < waypoints.size(); ++j) {
            if (i == j) {
                continue;
            }

            const std::string & wp_a = waypoints[i];
            const std::string & wp_b = waypoints[j];

            std::string predicate_str = "(connected " + wp_a + " " + wp_b + ")";

            bool already_connected =
                problem_expert->existPredicate(plansys2::Predicate(predicate_str));

            if (!already_connected) {
                std::string goal_str = "(and " + predicate_str + ")";
                goal_queue.push_back(goal_str);

                cout << "check_if_connected_list: aggiunto goal " << goal_str << endl;
            }
        }
    }
}

// ============================================================
// Funzione di supporto interna: trova la waypoint attuale del
// robot interrogando il predicato (robot_at ?robot ?wp).
// Ritorna stringa vuota se non trovata.
// ============================================================
static std::string get_robot_current_waypoint(
    const std::string & robot_name,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert)
{
    auto predicates = problem_expert->getPredicates();

    for (const auto & pred : predicates) {
        std::string pred_str = parser::pddl::toString(pred);

        if (pred_str.rfind("(robot_at " + robot_name + " ", 0) == 0) {
            std::string trimmed = pred_str.substr(0, pred_str.size() - 1); // rimuovo ')'
            size_t last_space = trimmed.find_last_of(' ');
            return trimmed.substr(last_space + 1);
        }
    }

    return "";
}

// ============================================================
// plan_patrol
// ------------------------------------------------------------
// 1. Chiama check_if_connected_list sulla lista fornita.
// 2. Interroga la knowledge base per la posizione attuale del
//    robot (robot_at).
// 3. Calcola l'ordine di visita con un semplice algoritmo
//    greedy nearest-neighbor: parte dalla posizione attuale e,
//    ad ogni passo, sceglie tra le waypoint non ancora visitate
//    quella con distanza minore (letta con get_connection).
// 4. Inserisce nella coda i goal (patrolled ?wp) nell'ordine
//    trovato.
//
// get_connection lancia std::runtime_error se la connessione
// richiesta non esiste: qui NON viene catturata, quindi in tal
// caso plan_patrol fallisce e basta, propagando l'eccezione al
// chiamante.
// ============================================================
void plan_patrol(
    const std::string & robot_name,
    const std::vector<std::string> & waypoints,
    std::deque<std::string> & goal_queue,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert)
{
    // 1. Assicura le connessioni necessarie
    check_if_connected_list(waypoints, goal_queue, problem_expert);

    // 2. Posizione attuale del robot
    std::string current_wp = get_robot_current_waypoint(robot_name, problem_expert);

    if (current_wp.empty()) {
        throw std::runtime_error(
            "plan_patrol: impossibile determinare la posizione attuale di " + robot_name);
    }

    // Lista delle waypoint ancora da visitare (escludo la posizione attuale)
    std::vector<std::string> to_visit;
    for (const auto & wp : waypoints) {
        if (wp != current_wp) {
            to_visit.push_back(wp);
        }
    }

    // 3. Greedy nearest-neighbor
    std::string from_wp = current_wp;
    std::vector<std::string> order;

    while (!to_visit.empty()) {
        size_t best_index = 0;
        float best_distance = -1.0f;

        for (size_t i = 0; i < to_visit.size(); ++i) {
            Connection cn = get_connection(from_wp, to_visit[i]); // lancia eccezione se manca

            if (best_distance < 0.0f || cn.distance < best_distance) {
                best_distance = cn.distance;
                best_index = i;
            }
        }

        std::string next_wp = to_visit[best_index];
        order.push_back(next_wp);

        to_visit.erase(to_visit.begin() + best_index);
        from_wp = next_wp;
    }

    // 4. Inserimento dei goal (patrolled ?wp) nell'ordine trovato
    for (const auto & wp : order) {
        std::string goal_str = "(and (patrolled " + wp + "))";
        goal_queue.push_back(goal_str);
        cout << "plan_patrol: aggiunto goal " << goal_str << endl;
    }
}



void print_goal_queue(const std::deque<std::string> & goal_queue)
{   
    cout << "GOAL_QUEUE:" << endl;
    for (const auto & goal : goal_queue) {
        cout << goal << endl;
    }
    cout << "__GOAL_QUEUE:" << endl;
}