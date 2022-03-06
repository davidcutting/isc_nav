// MIT License
//
// Copyright (c) Intelligent Systems Club
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "isc_nav/path_planner_node.hpp"
#include <isc_nav/utility/bfs.hpp>

namespace isc_nav
{

using namespace std::chrono_literals; // for 1000ms?

PathPlanner::PathPlanner(rclcpp::NodeOptions options)
: Node("path_planner", options)
{
    this->declare_parameter<std::string>("robot_frame", "base_footprint");
    this->declare_parameter<std::string>("map_frame", "map");
    param_update_timer_ = this->create_wall_timer(
      1000ms, std::bind(&PathPlanner::update_params, this)
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    transform_tolerance_ = tf2::durationFromSec(0.1);

    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 3,
        std::bind(&PathPlanner::map_callback, this, std::placeholders::_1)
    );

    goal_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/goal_pose", 10,
        std::bind(&PathPlanner::goal_callback, this, std::placeholders::_1)
    );

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/path", 5
    );
}

void PathPlanner::update_params()
{
    this->get_parameter("robot_frame", robot_frame_);
    this->get_parameter("map_frame", map_frame_);
}

void PathPlanner::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    last_map_state_ = msg;

    geometry_msgs::msg::PoseWithCovarianceStamped robot_pose{};
    geometry_msgs::msg::PoseWithCovarianceStamped transformed_pose{};
    robot_pose.header.frame_id = map_frame_;
    robot_pose.header.stamp = this->get_clock()->now();

    try
    {   // perform the tf transform and publish the resulting pose
        transformed_pose = tf_buffer_->transform(robot_pose, "map", transform_tolerance_);
        *last_pos_state_ = transformed_pose.pose.pose;
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            map_frame_.c_str(), robot_frame_.c_str(), ex.what()
        );
        return;
    }

    if (last_goal_state_ != nullptr && last_pos_state_ != nullptr)
    {
        auto bfs = BreadthFirstSearch(*last_map_state_);
        bfs.set_start(*last_pos_state_);
        bfs.set_goal(*last_goal_state_);
        path_publisher_->publish(bfs.get_path());
    }
}

void PathPlanner::goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    last_goal_state_ = msg;
}

}  // namespace isc_nav

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto path_node = std::make_shared<isc_nav::PathPlanner>(options);
  exec.add_node(path_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}