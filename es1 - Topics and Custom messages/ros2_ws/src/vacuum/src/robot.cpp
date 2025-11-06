#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp" 
#include "vac_interfaces/msg/status.hpp"

using namespace std::chrono_literals;


// Rooms of computer vision lab.
const std::vector<std::string> ROOMS = {
    "robot_vision_lab",
    "ssl_lab",
    "neurorob_lab",
    "ias_lab",
    "autonomous_rob_lab"
};


class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() 
    : Node("minimal_publisher"),
      counter_{0},
      battery_{100},
      room_idx_{0}
    {
        publisher_ = 
	        this->create_publisher<vac_interfaces::msg::Status>(
	            "topic", 10 
	        );

        auto timer_callback =
            [this]() -> void {

                // Update battery, if necessary.
                if (!(++counter_ % 5) && battery_ > 0) battery_ -= 1;

                // Change room if you have finished cleaning the current one.
                bool chg_room = (!(counter_ % 15))? 1 : 0;

                // If you have enough battery and you have finished cleaning
                // current room, then change room.
                if (battery_ && chg_room) {
                    size_t rnd;
                    while((rnd = rand() % ROOMS.size()) == room_idx_);
                    room_idx_ = rnd;
                }

                RCLCPP_INFO(
                    this->get_logger(), 
                    "Sending status...\n\t room_idx_=%lu",
                    room_idx_
                );

                // Create the message.
                auto message = vac_interfaces::msg::Status();
                message.id = room_idx_ + 1;
                message.name = ROOMS[room_idx_];
                message.battery = battery_;

                this->publisher_->publish(message);
            };

        timer_ = this->create_wall_timer(0.2s, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<vac_interfaces::msg::Status>::SharedPtr publisher_;
    size_t counter_;
    long int battery_;
    size_t room_idx_;
};


int main(int argc, char * argv[]) {
    srand(time(0));
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
