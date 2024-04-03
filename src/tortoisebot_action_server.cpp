#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
//custom service
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
// content of the action interface
// # goal definition
// geometry_msgs/Point position
// ---
// # result definition
// bool success
// ---
// # feedback
// geometry_msgs/Point position
// string state

using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<WaypointAction>;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class WaypointActionClass : public rclcpp::Node
{
public:
    WaypointActionClass() : Node("tortoisebot_as")
    {
        // Action server
        action_server_ = rclcpp_action::create_server<WaypointAction>(
            this, "tortoisebot_as",
            std::bind(&WaypointActionClass::handle_goal, this, _1, _2),
            std::bind(&WaypointActionClass::handle_cancel, this, _1),
            std::bind(&WaypointActionClass::handle_accepted, this, _1));

        // Publishers and Subscribers
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&WaypointActionClass::clbk_odom, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Action server started");
    }

private:
    // Action server
    rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    // Robot state variables
    geometry_msgs::msg::Point position_;
    double yaw_;

    // Goal and parameters
    geometry_msgs::msg::Point des_pos_;
    double yaw_precision_ = M_PI / 90; // +/- 2 degrees allowed
    double dist_precision_ = 0.05;

    // State
    std::string state_;

    // Feedback and Result
    auto feedback_ = std::make_shared<WaypointAction::Feedback>();
    auto result_ = std::make_shared<WaypointAction::Result>();

    void clbk_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Position
        position_ = msg->pose.pose.position;

        // Yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw, 1);
        // store the current yaw_
        yaw_ = yaw;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const WaypointAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::GoalResponse handle_accepted(
        const std::shared_ptr<GoalHandleWaypoint> goal_handle)
    {
        auto pos = goal_handle->get_goal()->position;
        RCLCPP_INFO(this->get_logger(), "Goal (%f, %f, %f) received", pos.x, pos.y, pos.z);

        // Initializations
        rclcpp_action::GoalResponse res = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        auto goal = goal_handle->get_goal();

        // Define desired position and errors
        des_pos_ = goal->position;
        double desired_yaw = std::atan2(des_pos_.y - position_.y, des_pos_.x - position_.x);
        double err_pos = std::sqrt(std::pow(des_pos_.y - position_.y, 2) + std::pow(des_pos_.x - position_.x, 2));
        double err_yaw = desired_yaw - yaw_;

        // Perform task
        while (err_pos > dist_precision_)
        {
            // Update variables
            desired_yaw = std::atan2(des_pos_.y - position_.y, des_pos_.x - position_.x);
            err_yaw = desired_yaw - yaw_;
            err_pos = std::sqrt(std::pow(des_pos_.y - position_.y, 2) + std::pow(des_pos_.x - position_.x, 2));
            RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", yaw_);
            RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f", desired_yaw);
            RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", err_yaw);

            // Logic
            if (goal_handle->is_canceling())
            {
                // Cancelled
                RCLCPP_INFO(this->get_logger(), "The goal has been cancelled/preempted");
                result_.success = false;
                goal_handle->canceled(result_);
                res = rclcpp_action::GoalResponse::REJECT;
                break;
            }
            else if (std::fabs(err_yaw) > yaw_precision_)
            {
                // Fix yaw
                RCLCPP_INFO(this->get_logger(), "fix yaw");
                state_ = "fix yaw";
                auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
                twist_msg->angular.z = err_yaw > 0 ? 0.65 : -0.65;
                pub_cmd_vel_->publish(std::move(twist_msg));
            }
            else
            {
                // Go to point
                RCLCPP_INFO(this->get_logger(), "go to point");
                state_ = "go to point";
                auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
                twist_msg->linear.x = 0.6;
                twist_msg->angular.z = 0;
                pub_cmd_vel_->publish(std::move(twist_msg));
            }

            // Send feedback
            feedback_.position = position_;
            feedback_.state = state_;
            goal_handle->publish_feedback(feedback_);

            // Loop rate
            rclcpp::sleep_for(std::chrono::milliseconds(40));
        }

        // Stop
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 0;
        twist_msg->angular.z = 0;
        pub_cmd_vel_->publish(std::move(twist_msg));

        // Return result
        if (res == rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE)
        {
            result_.success = true;
            goal_handle->succeed(result_);
        }

        return res;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWaypoint> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointActionClass>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
