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

#include "../path_planner_node.hpp"
#include <isc_nav/utility/bfs.hpp>
#include <isc_nav/utility/rrt.hpp>

namespace isc_nav
{

using namespace std::chrono_literals; // for 1000ms?

PathPlanner::PathPlanner(rclcpp::NodeOptions options)
: Node("path_planner", options), last_map_state_{}, last_goal_state_{}, last_pos_state_{}
{
    this->declare_parameter<std::string>("robot_frame", "base_footprint");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<float>("tf_timeout", 0.5f);
    update_params();
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

    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
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
    this->get_parameter("tf_timeout", tf_timeout_);
    transform_tolerance_ = tf2::durationFromSec(tf_timeout_);
}

void PathPlanner::update_plan()
{
    geometry_msgs::msg::PoseWithCovarianceStamped robot_pose{};
    geometry_msgs::msg::PoseWithCovarianceStamped transformed_pose{};
    robot_pose.header.frame_id = robot_frame_;
    robot_pose.header.stamp = this->get_clock()->now();

    try
    {
        // get robot pose in map
        transformed_pose = tf_buffer_->transform(robot_pose, map_frame_, transform_tolerance_);
        last_pos_state_ = std::make_shared<geometry_msgs::msg::Pose>(transformed_pose.pose.pose);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            map_frame_.c_str(), robot_frame_.c_str(), ex.what()
        );
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Updating plan.");

    /*
    auto bfs = BreadthFirstSearch(*last_map_state_);
    nav_msgs::msg::Path bfs_path = bfs.get_path(*last_pos_state_, last_goal_state_->pose);
    
    bfs_path.header.frame_id = map_frame_;
    bfs_path.header.stamp = this->get_clock()->now();
    path_publisher_->publish(bfs_path);
    */

    auto rrt = RRT(*last_map_state_);
    nav_msgs::msg::Path rrt_path = rrt.get_path(*last_pos_state_, last_goal_state_->pose);
    
    rrt_path.header.frame_id = map_frame_;
    rrt_path.header.stamp = this->get_clock()->now();
    path_publisher_->publish(rrt_path);

}

void PathPlanner::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received a new map.");
    last_map_state_ = msg;

    if (last_goal_state_ == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Skipping plan update because we're missing goal info.");
        return;
    }

    update_plan();
}

void PathPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received a new goal pose.");
    geometry_msgs::msg::PoseStamped transformed_pose{};

    try
    {
        // transform goal to map frame if it isn't
        transformed_pose = tf_buffer_->transform(*msg, map_frame_, transform_tolerance_);
        last_goal_state_ = std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_pose);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            map_frame_.c_str(), msg->header.frame_id.c_str(), ex.what()
        );
        return;
    }

    if (last_map_state_ == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Skipping plan update because we're missing map info.");
        return;
    }

    update_plan();
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