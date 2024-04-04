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
    WaypointActionClient() : Node("waypoint_action_client"),
                            goal_done_(false),
                            result_pos_(false),
                            result_yaw_(false)
    {
        this->client_ptr_ = rclcpp_action::create_client<WaypointAction>(
            this,
            "tortoisebot_as");
    }

    bool is_server_available()
    {
        return this->client_ptr_->wait_for_action_server(std::chrono::seconds(10));
    }

    std::shared_future<GoalHandleWaypoint::SharedPtr> send_goal(double x, double y, double z)
    {
        this->goal_done_ = false;
        this->result_pos_ = false;
        this->result_yaw_ = false;

        auto goal_msg = WaypointAction::Goal();
        goal_msg.position.x = x;
        goal_msg.position.y = y;
        goal_msg.position.z = z;

        auto send_goal_options = rclcpp_action::Client<WaypointAction>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WaypointActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&WaypointActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&WaypointActionClient::result_callback, this, std::placeholders::_1);
        return this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    bool is_goal_done() const { return goal_done_; }
    bool get_result_pos() const { return result_pos_; }
    bool get_result_yaw() const { return result_yaw_; }
    
private:
    rclcpp_action::Client<WaypointAction>::SharedPtr client_ptr_;
    bool goal_done_;
    bool result_pos_;
    bool result_yaw_;

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

    void result_callback(const GoalHandleWaypoint::WrappedResult & result)
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                this->result_pos_ = result.result->success_pos;
                this->result_yaw_ = result.result->success_yaw;
                this->goal_done_ = true;
                // // log the result
                // RCLCPP_INFO(this->get_logger(), "Result Pos: %s", this->result_pos_ ? "Success" : "Failure");
                // RCLCPP_INFO(this->get_logger(), "Result Yaw: %s", this->result_yaw_ ? "Success" : "Failure");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Unknown result code");
                break;
        }
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
    while(!action_client->is_goal_done()) {
        rclcpp::spin_some(action_client);
    }
    // show the result
    RCLCPP_INFO(action_client->get_logger(), "Result Pos: %s", action_client->get_result_pos() ? "Success" : "Failure");
    RCLCPP_INFO(action_client->get_logger(), "Result Yaw: %s", action_client->get_result_yaw() ? "Success" : "Failure");

    action_client->send_goal(0.0, 0.5, 0.0);
    while(!action_client->is_goal_done()) {
        rclcpp::spin_some(action_client);
    }
    // show the result
    RCLCPP_INFO(action_client->get_logger(), "Result Pos: %s", action_client->get_result_pos() ? "Success" : "Failure");
    RCLCPP_INFO(action_client->get_logger(), "Result Yaw: %s", action_client->get_result_yaw() ? "Success" : "Failure");

    rclcpp::shutdown();
    return 0;
}