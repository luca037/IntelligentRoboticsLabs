#include <iostream>
#include <memory>
#include <cstdlib>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/refill_burrow.hpp" 


class BurrowClient : public rclcpp::Node {

 public:

    using RefillBurrow = interfaces::srv::RefillBurrow;

    BurrowClient() : Node("burrow_client") {
        // Init the client.
        client_ = this->create_client<RefillBurrow>("refill_burrow");
        
        // Seed random number generator.
        std::srand(std::time(NULL));

        // Send request periodically.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BurrowClient::send_burrow_request, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Burrow Service Client is ready."
        );
    }

 private:
    rclcpp::Client<RefillBurrow>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_burrow_request() {
        // Wait for the service to be available.
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(
                this->get_logger(),
                "Service not available, waiting..."
            );
            return;
        }

        // Generate random n and s.
        int s = std::rand() % 11 + 10; // rand in [10, 20]
        int n = std::rand() % s; // rand in [0, 20]
        
        // Create request.
        auto request = std::make_shared<RefillBurrow::Request>();
        request->n = n;
        request->s = s;

        RCLCPP_INFO(
            this->get_logger(), 
            "Sending request: n=%d, s=%d", 
            request->n, request->s
        );

        // Send the request.
        auto result = 
            client_->async_send_request(
                request, 
                std::bind(
                    &BurrowClient::handle_response,
                    this,
                    std::placeholders::_1
                )
            );
    }

    void handle_response(
        rclcpp::Client<RefillBurrow>::SharedFuture result
    ) {
        if (result.valid())
            RCLCPP_INFO(
                this->get_logger(), 
                "Turtlebot Response: %s", 
                result.get()->success? 
                "Enough apples found!" : "NOT enough apples!"
            );
        else
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to call service 'refill_burrow'"
            );
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BurrowClient>());
    rclcpp::shutdown();
    return 0;
}
