#pragma once

#include "isc_nav/pure_pursuit.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace isc_nav
{
class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuit::PurePursuit m_tracker;
    explicit PurePursuitNode(rclcpp::NodeOptions options);

private:
    Path m_path;
    Point3D pose;
    std::string robot_frame;
    std::string map_frame;

    double m_lookahead_distance;
    bool m_path_is_initialized;
    bool is_initialized;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    Path to_path(const nav_msgs::msg::Path::SharedPtr msg);
    Point3D to_point3d(const geometry_msgs::msg::PoseStamped pose);
    Point3D get_pose();

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
};
}  // namespace isc_nav