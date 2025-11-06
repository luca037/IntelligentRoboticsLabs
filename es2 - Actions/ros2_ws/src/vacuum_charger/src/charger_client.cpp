#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "vacuum_action_interfaces/action/charging.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace vacuum_charger {

class ChargerActionClient: public rclcpp::Node {
public:
    using Charging = vacuum_action_interfaces::action::Charging;
    using GoalHandleCharging = rclcpp_action::ClientGoalHandle<Charging>;

    explicit ChargerActionClient(const rclcpp::NodeOptions& options)
    : Node("charger_action_client", options) {
        // Define an action client.
        this->client_ptr_ = 
            rclcpp_action::create_client<Charging>(
                this,
                "charging"
            );

        // Instantiate a ROS timer that will kick off the 
        // one and only call to send_goal function.
        auto timer_callback_lambda = 
	        [this](){ return this->send_goal(); };
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            timer_callback_lambda
        );
    }

    void send_goal() {
        using namespace std::placeholders;

        // Cancels the timer (so it is only called once).
        this->timer_->cancel();

        // Waits for the action server to come up.
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Action server not available after waiting"
            );
            rclcpp::shutdown();
        }

        // Define a (Charging) goal message.
        auto goal_msg = Charging::Goal();
        goal_msg.power = (uint) (60 + rand() % 40);

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        // Define Response, Feedback and Result callbacks.
        auto send_goal_options = 
            rclcpp_action::Client<Charging>::SendGoalOptions();
        
        // ### Response callback ###.
        send_goal_options.goal_response_callback = 
            [this](const GoalHandleCharging::SharedPtr& goal_handle) {
                if (!goal_handle)
                    RCLCPP_ERROR(
	                    this->get_logger(),
	                    "Goal was rejected by server"
	                );
                else
                    RCLCPP_INFO(
	                    this->get_logger(), 
	                    "Goal accepted by server,"
	                    "waiting for result"
	                );
            };

        // ### Feedback callback ###.
        send_goal_options.feedback_callback = 
            [this](
                GoalHandleCharging::SharedPtr,
                const std::shared_ptr<const Charging::Feedback> feedback
            ) {
                // Log current battery of vacuum.
                RCLCPP_INFO(
	                this->get_logger(),
                    "Current battery level: %u", feedback->curr_battery
	            );
            };

        // ### Result callback ###.
        send_goal_options.result_callback = 
            [this](
	            const GoalHandleCharging::WrappedResult& result
	        ) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(
	                    this->get_logger(),
	                    "Goal was aborted"
	                );
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(
	                    this->get_logger(),
	                    "Goal was canceled"
	                );
                    return;
                default:
                    RCLCPP_ERROR(
	                    this->get_logger(),
	                    "Unknown result code"
	                );
                    return;
                }
                // Log result received.
                RCLCPP_INFO(
	                this->get_logger(),
                    "Final battery level: %u", result.result->new_battery
                );
                rclcpp::shutdown();
            };

        // Send goal to the server.
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
  rclcpp_action::Client<Charging>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

};  // class ChargingActionClient

}  // namespace vacuum_charger

RCLCPP_COMPONENTS_REGISTER_NODE(vacuum_charger::ChargerActionClient)
