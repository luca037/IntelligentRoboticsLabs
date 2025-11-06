#include <functional>
#include <memory>
#include <thread>
#include <algorithm>

#include "vacuum_charger/visibility_control.h"
#include "vacuum_action_interfaces/action/charging.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace vacuum_charger {

class VacuumActionServer: public rclcpp::Node {
public:

    using Charging = vacuum_action_interfaces::action::Charging;
    using GoalHandleCharging = rclcpp_action::ServerGoalHandle<Charging>;

    CUSTOM_ACTION_CPP_PUBLIC
    explicit VacuumActionServer(
		const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
    ) : Node( "vacuum_action_server", options) {
        using namespace std::placeholders;

        // Just accept all new goals.
        auto handle_goal = 
            [this](
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const Charging::Goal> goal
            ) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Received goal: power supplied %u",
                    goal->power
                );
                (void)uuid;  // prevent -Wunused warnings.
                return rclcpp_action::
                GoalResponse::ACCEPT_AND_EXECUTE;
            };

        // Just accept all cancellation.
        auto handle_cancel = 
            [this](
	            const std::shared_ptr<GoalHandleCharging> 
	            goal_handle
            ) {
                RCLCPP_INFO(
	                this->get_logger(),
	                "Received request to cancel goal"
		        );
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

        // Start processing the goal.
        auto handle_accepted = 
            [this](
	            const std::shared_ptr<GoalHandleCharging> goal_handle
	        ) {
                auto execute_in_thread = 
                    [this, goal_handle]() {
                        // See 'execute' function below.
                        return this->execute(goal_handle);
                    };
                std::thread{execute_in_thread}.detach();
            };

        // Define an action server.
        this->action_server_ = 
	    rclcpp_action::create_server<Charging>(
            this,
            "charging",
            handle_goal,
            handle_cancel,
            handle_accepted
        );
    }

private:
    rclcpp_action::Server<Charging>::SharedPtr action_server_;

    // Charging operation.
    void execute(
	    const std::shared_ptr<GoalHandleCharging> goal_handle
    ) {
        // Define goal.
        const auto goal = goal_handle->get_goal();
        const uint power = goal->power;

        // Define feedback.
        auto feedback = std::make_shared<Charging::Feedback>();
        // Init current battery: random between 0-8.
        feedback->curr_battery = rand() % 8;

        // Define the result object.
        auto result = std::make_shared<Charging::Result>();

        // Compute the next element in Charging sequence.
        const uint final_power = 
            std::min(100u, power + feedback->curr_battery);

        // One itration per second.
        rclcpp::Rate loop_rate(final_power / 60.);

        // Define a function used to publish feedback.
        auto publish_callback_lambda =
            [this, goal_handle, feedback]() {
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(),"Publish feedback");
            };

        rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(
            std::chrono::seconds(1), // Update feedback every second.
            publish_callback_lambda
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Executing goal...\n\tstarting level: %u, final level: %u",
            feedback->curr_battery, final_power
        );

        // Charging loop.
        const int itr = final_power - feedback->curr_battery;
        for (int i = 0; (i < itr)&& rclcpp::ok(); ++i) {

            // Check if there is a cancel request.
            if (goal_handle->is_canceling()) {
                result->new_battery = feedback->curr_battery;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Increase battery.
            feedback->curr_battery += 1;

            loop_rate.sleep(); // Wait.
        }

        // Check if goal is done.
        if (rclcpp::ok()) {
            // Save result.
            result->new_battery = feedback->curr_battery;
            goal_handle->succeed(result);
            RCLCPP_INFO(
	            this->get_logger(), 
	            "Goal succeeded"
            );
        }
    };

};  // class VacuumActionServer

}  // namespace vacuum_charger

RCLCPP_COMPONENTS_REGISTER_NODE(vacuum_charger::VacuumActionServer)
