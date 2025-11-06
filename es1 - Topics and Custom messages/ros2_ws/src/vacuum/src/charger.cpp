#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vac_interfaces/msg/status.hpp"


class MinimalSubscriber : public rclcpp::Node {

public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        
        auto topic_callback = 
            [this](vac_interfaces::msg::Status::UniquePtr msg) {
                RCLCPP_INFO(
	                this->get_logger(),
	                "\n\tReceived->  id: %ld, name: %s, battery: %ld", 
                    msg->id,
	                msg->name.c_str(),
                    msg->battery
	            );
            };

        subscription_ =
            this->create_subscription<vac_interfaces::msg::Status>(
                "topic", 10, topic_callback
            );
    }

private:
    rclcpp::Subscription<vac_interfaces::msg::Status>::SharedPtr subscription_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
