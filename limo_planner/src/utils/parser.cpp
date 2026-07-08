#include <string>
#include <vector>
#include <cstdio>
#include <iostream>  // std::cout, std::endl
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "controller_functions.hpp"

#include "parser.hpp"

using namespace std;


// std::string trim(std::string s){
//     // trovo dove inizia il testo vero, saltando gli spazi iniziali
//     int start = 0;
//     while (start < s.size() && s[start] == ' ') {
//         start++;
//     }

//     // trovo dove finisce il testo vero, saltando gli spazi finali
//     int end = s.size() - 1;
//     while (end >= start && s[end] == ' ') {
//         end--;
//     }

//     // ricostruisco la stringa solo con la parte "buona"
//     std::string result = "";
//     for (int i = start; i <= end; i++) {
//         result += s[i];
//     }

//     return result;
// }


std::string trim(std::string s){
    auto is_space = [](unsigned char c) {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n';
    };

    int start = 0;
    while (start < (int)s.size() && is_space(s[start])) {
        start++;
    }

    int end = (int)s.size() - 1;
    while (end >= start && is_space(s[end])) {
        end--;
    }

    return s.substr(start, end - start + 1);
}



std::vector<std::string> split_string(const std::string & s, char delim){
    std::vector<std::string> parts;
    std::string current = "";
    for (char c : s) {
        if (c == delim) {
            parts.push_back(current);
            current = "";
        } else {
            current += c;
        }
    }
    parts.push_back(current);
    return parts;
}

// da "pred1, pred2" costruisce "(and (pred1) (pred2))"
std::string build_and_goal(const std::string & group){
    std::vector<std::string> preds = split_string(group, ',');
    std::string goal = "(and";
    for (auto p : preds) {
        p = trim(p);
        if (p != "") {
            goal += " (" + p + ")";
        }
    }
    goal += ")";
    return goal;
}
