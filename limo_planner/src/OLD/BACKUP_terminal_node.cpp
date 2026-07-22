#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

class GoalInputPublisher : public rclcpp::Node
{
public:
    GoalInputPublisher()
    : rclcpp::Node("goal_input_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("goal_input", 10);
    }

    // Legge righe da stdin (con getline, quindi accetta spazi) e pubblica
    // ogni riga appena l'utente preme Invio.
    void run()
    {
        cout << "Publisher pronto. Scrivi il goal (o i goal, separati da '>') e premi INVIO per pubblicare." << endl;
        cout << "Premi Ctrl+D (o Ctrl+Z su Windows) per uscire." << endl;

        std::string line;
        while (rclcpp::ok() && std::getline(std::cin, line)) {
            if (line.empty()) {
                // riga vuota: non pubblichiamo nulla, si ricomincia
                continue;
            }

            auto msg = std_msgs::msg::String();
            msg.data = line;

            publisher_->publish(msg);
            cout << "Pubblicato su 'goal_input': \"" << line << "\"" << endl;
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