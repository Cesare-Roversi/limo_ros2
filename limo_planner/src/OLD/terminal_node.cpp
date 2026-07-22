#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "parser.hpp"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"


using namespace std;

class GoalInputPublisher : public rclcpp::Node
{
public:
  GoalInputPublisher()
  : rclcpp::Node("goal_input_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("goal_input", 10);
  }

  // Legge righe da stdin (con getline, quindi accetta spazi).
  // Ogni riga puo' contenere piu' goal separati da '>' (sequenza),
  // e ogni goal puo' contenere piu' predicati separati da ',' (and).
  void run()
  {
    cout << "Publisher pronto. Scrivi il goal (o i goal, separati da '>') e premi INVIO per pubblicare." << endl;
    cout << "Usa ',' per unire piu' predicati in un singolo goal, es: pred1, pred2 > pred3" << endl;

    std::string line;
    while (rclcpp::ok() && std::getline(std::cin, line)) {
      if (line.empty()) {
        // riga vuota: non pubblichiamo nulla, si ricomincia
        continue;
      }

      // divide la riga in goal separati da '>'
      std::vector<std::string> goal_groups = split_string(line, '>');
      bool published_any = false;

      for (const auto & group : goal_groups) {
        std::string goal_str = build_and_goal(group);

        if (goal_str == "(and)") {
          // gruppo vuoto (es. '>' consecutivi o solo spazi): lo saltiamo
          continue;
        }

        auto msg = std_msgs::msg::String();
        msg.data = goal_str;
        publisher_->publish(msg);
        published_any = true;

        cout << "Pubblicato su 'goal_input': \"" << goal_str << "\"" << endl;
      }

      if (!published_any) {
        cout << "Nessun goal valido trovato nella riga inserita." << endl;
      }
    }
    cout << "Chiusura publisher (EOF o shutdown)." << endl;
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalInputPublisher>();
  // rclcpp::spin non serve qui perché non riceviamo callback: leggiamo
  // semplicemente da stdin in modo bloccante nel main thread.
  node->run();
  rclcpp::shutdown();
  return 0;
}