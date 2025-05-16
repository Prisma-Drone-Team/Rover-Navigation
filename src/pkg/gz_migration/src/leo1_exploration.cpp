#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;

class MultiGoalSender : public rclcpp::Node {
public:
    MultiGoalSender() : Node("multi_goal_sender"), goal_index_(0) {
        // Inizializza l'action client per NavigateToPose
        client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/robot1/navigate_to_pose");

        // Attendi che l'action server sia disponibile
        if (!client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server non disponibile!");
            rclcpp::shutdown();
        }

        // Definisci più pose da raggiungere
        add_goal(-0.5, 0.5);
        add_goal(-1.0, 1.0);
        add_goal(-1.0, 1.5);
        add_goal(-1.0, 2.0);

        // int i = 1;
        // for (int x = 0; x < 10; x++){  // x va da -1 a 9
        //     if(i==1){
        //         for (double y = 2; y >= -3; y -= 0.5) {
        //             add_goal(x, y);
        //         }
        //         i = -i;  // Alterna il segno di i
        //     }
        //     else{
        //         for (double y = -3; y <= 2; y += 0.5) {
        //             add_goal(x, y);
        //         }
        //         i = -i;  // Alterna il segno di i
        //     }
        // }
        int i = 1;
        for (int y = 2; y > -8; y--){  // y va da 2 a -8
            if(i==1){
                for (double x = -2; x <= 3; x += 0.5) {
                    add_goal(x, y);
                }
                i = -i;  // Alterna il segno di i
            }
            else{
                for (double x = 3; x >= -2; x -= 0.5) {
                    add_goal(x, y);
                }
                i = -i;  // Alterna il segno di i
            }
        }
        send_next_goal();
    };


private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    std::vector<nav2_msgs::action::NavigateToPose::Goal> goals_;
    size_t goal_index_;
    rclcpp::TimerBase::SharedPtr goal_timer_;

    // Funzione per aggiungere un goal alla lista
    void add_goal(double x, double y) {
        nav2_msgs::action::NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "robot1/map";
        goal.pose.header.stamp = this->get_clock()->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        //goal.pose.pose.orientation.w = w;
        goals_.push_back(goal);
    }

    // Funzione per inviare il prossimo goal
    void send_next_goal() {
        if (goal_index_ >= goals_.size()) {
            RCLCPP_INFO(this->get_logger(), "Tutti i goal sono stati raggiunti!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Invio goal alla posizione (%.1f, %.1f)", 
                    goals_[goal_index_].pose.pose.position.x, goals_[goal_index_].pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](auto) {
            RCLCPP_INFO(this->get_logger(), "Goal accettato!");
            //goal_timer();  // Avvia il timer per il goal
        };
        send_goal_options.result_callback = [this](const auto& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Goal raggiunto!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Fallimento nel raggiungere il goal.");
            }
            goal_index_++;  // Passa al goal successivo
            send_next_goal();  // Invia il prossimo goal
        };

        client_->async_send_goal(goals_[goal_index_], send_goal_options);
    }

    // void goal_timer(){
    //     goal_timer_ = this->create_wall_timer(20s, [this](){
    //         RCLCPP_INFO(this->get_logger(), "Goal non raggiunto in 20s, reinvio il goal");
    //         goal_index_=goal_index_-1;
    //         send_next_goal();
    //     });
    // }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiGoalSender>());
    rclcpp::shutdown();
    return 0;
}

