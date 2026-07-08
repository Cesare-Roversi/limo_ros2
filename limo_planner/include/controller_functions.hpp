#ifndef CONTROLLER_FUNCTIONS_HPP
#define CONTROLLER_FUNCTIONS_HPP

#include <deque>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

// ============================================================
// check_if_connected_list
// ------------------------------------------------------------
// Itera su tutte le coppie ordinate (wp_i, wp_j) con i != j
// della lista fornita. Per ogni coppia il cui predicato
// (connected wp_i wp_j) non è ancora istanziato nella problem
// expert, inserisce un goal corrispondente nella coda.
// ============================================================
void check_if_connected_list(
    const std::vector<std::string> & waypoints,
    std::deque<std::string> & goal_queue,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert);

// ============================================================
// plan_patrol
// ------------------------------------------------------------
// 1. Chiama check_if_connected_list sulla lista fornita.
// 2. Interroga la knowledge base per la posizione attuale del
//    robot (robot_at).
// 3. Calcola l'ordine di visita con un algoritmo greedy
//    nearest-neighbor basato su get_connection.
// 4. Inserisce nella coda i goal (patrolled ?wp) nell'ordine
//    trovato.
//
// Se get_connection lancia std::runtime_error perché una
// connessione richiesta non esiste, l'eccezione NON viene
// catturata qui: plan_patrol fallisce e basta, propagandola
// al chiamante.
// ============================================================
bool plan_patrol(
    const std::string & robot_name,
    const std::vector<std::string> & waypoints,
    std::deque<std::string> & goal_queue,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert);


void print_goal_queue(const std::deque<std::string> & goal_queue);

#endif // CONTROLLER_FUNCTIONS_HPP