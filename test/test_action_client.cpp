#include "gtest/gtest.h"
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
                            goal_done_(false)
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

    bool is_goal_done() const
    {
        return this->goal_done_;
    }
    
private:
    rclcpp_action::Client<WaypointAction>::SharedPtr client_ptr_;
    bool goal_done_;

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
                this->goal_done_ = true;
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

class WaypointActionClientTest : public ::testing::Test {
public:
  WaypointActionClientTest() {
    rclcpp::init(0, nullptr);
    action_client_ = std::make_shared<WaypointActionClient>();
  }

  ~WaypointActionClientTest() {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<WaypointActionClient> action_client_;
};

TEST_F(WaypointActionClientTest, TestGoalCompletion1) {
  EXPECT_TRUE(action_client_->is_server_available());
  // log if server is available or not
  if (action_client_->is_server_available()) {
    RCLCPP_INFO(action_client_->get_logger(), "~~~~~Server is available");
  } else {
    RCLCPP_ERROR(action_client_->get_logger(), "~~~~~Server is not available");
  }
  action_client_->send_goal(0.5, 0.0, 0.0);
  // log goal sent content
  RCLCPP_INFO(action_client_->get_logger(), "~~~~~Goal sent");
  while(!action_client_->is_goal_done()) {
    rclcpp::spin_some(action_client_);
  }
  EXPECT_TRUE(action_client_->is_goal_done());
  // log if goal was done or not
  if (action_client_->is_goal_done()) {
    RCLCPP_INFO(action_client_->get_logger(), "~~~~~Goal is done");
  } else {
    RCLCPP_ERROR(action_client_->get_logger(), "~~~~~Goal is not done");
  }
}

TEST_F(WaypointActionClientTest, TestGoalCompletion2) {
  EXPECT_TRUE(action_client_->is_server_available());
  // log if server is available or not
  if (action_client_->is_server_available()) {
    RCLCPP_INFO(action_client_->get_logger(), "~~~~~Server is available");
  } else {
    RCLCPP_ERROR(action_client_->get_logger(), "~~~~~Server is not available");
  }
  action_client_->send_goal(0.0, 0.5, 0.0);
  // log goal sent content
  RCLCPP_INFO(action_client_->get_logger(), "~~~~~Goal sent");
  while(!action_client_->is_goal_done()) {
    rclcpp::spin_some(action_client_);
  }
  EXPECT_TRUE(action_client_->is_goal_done());
  // log if goal was done or not
  if (action_client_->is_goal_done()) {
    RCLCPP_INFO(action_client_->get_logger(), "~~~~~Goal is done");
  } else {
    RCLCPP_ERROR(action_client_->get_logger(), "~~~~~Goal is not done");
  }
}
