#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include <memory>
#include <chrono>

using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<WaypointAction>;

class WaypointActionClient : public rclcpp::Node
{
public:
    WaypointActionClient() : Node("waypoint_action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<WaypointAction>(
            this,
            "tortoisebot_as");
    }

    bool is_server_available()
    {
        return this->client_ptr_->wait_for_action_server(std::chrono::seconds(10));
    }

    void send_goal(double x, double y, double z)
    {
        auto goal_msg = WaypointAction::Goal();
        goal_msg.position.x = x;
        goal_msg.position.y = y;
        goal_msg.position.z = z;

        auto send_goal_options = rclcpp_action::Client<WaypointAction>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WaypointActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&WaypointActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        // Send the goal and get a future for the result
        auto future_result = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Goal was successful");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to complete goal");
        }
    }

private:
    rclcpp_action::Client<WaypointAction>::SharedPtr client_ptr_;

    void goal_response_callback(std::shared_future<GoalHandleWaypoint::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleWaypoint::SharedPtr,
        const std::shared_ptr<const WaypointAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->state.c_str());
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<WaypointActionClient>();

    if (!action_client->is_server_available()) {
        RCLCPP_ERROR(action_client->get_logger(), "Action server is not available. Exiting.");
        return 0;
    }

    // Send two goals
    action_client->send_goal(0.5, 0.0, 0.0);
    action_client->send_goal(0.0, 0.5, 0.0);

    rclcpp::shutdown();
    return 0;
}