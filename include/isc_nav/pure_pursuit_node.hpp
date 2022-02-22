#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/twist.hpp>

namespace isc_nav
{
class PurePursuitNode : public rclcpp::Node
{
public:
    explicit PurePursuitNode(rclcpp::NodeOptions options);

private:
    nav_msgs::msg::Path::SharedPtr last_path_state_;

    /**
    * @brief Callback that will convert ros path to Point
    * @param the path message 
    * @return ??
    */
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
};
}  // namespace isc_nav