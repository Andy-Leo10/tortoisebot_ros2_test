#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class WaypointClient : public rclcpp::Node
{
public:
    WaypointClient() : Node("waypoint_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<tortoisebot_waypoints::action::WaypointAction>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "tortoisebot_as");
    }

    void send_goal(double x, double y, double z)
    {
        auto goal_msg = tortoisebot_waypoints::action::WaypointAction::Goal();
        goal_msg.position.x = x;
        goal_msg.position.y = y;
        goal_msg.position.z = z;

        auto send_goal_options = rclcpp_action::Client<tortoisebot_waypoints::action::WaypointAction>::SendGoalOptions();

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<tortoisebot_waypoints::action::WaypointAction>::SharedPtr client_ptr_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto waypoint_client = std::make_shared<WaypointClient>();

    RCLCPP_INFO(waypoint_client->get_logger(), "Moving to waypoint: (0.5, 0.5, 0.0)");
    waypoint_client->send_goal(0.5, 0.5, 0.0);

    // RCLCPP_INFO(waypoint_client->get_logger(), "Moving to waypoint: (0.25, 0.25, 0.0)");
    // waypoint_client->send_goal(0.25, 0.25, 0.0);

    // RCLCPP_INFO(waypoint_client->get_logger(), "Moving to waypoint: (-0.25, -0.25, 0.0)");
    // waypoint_client->send_goal(-0.25, -0.25, 0.0);

    rclcpp::spin(waypoint_client);
    rclcpp::shutdown();

    return 0;
}