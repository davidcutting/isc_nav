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

#include "isc_snav/path_planner_node.hpp"

namespace isc_snav
{

PathPlanner::PathPlanner(rclcpp::NodeOptions options)
: Node("path_planner", options)
{
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 3,
        std::bind(&PathPlanner::map_callback, this, std::placeholders::_1)
    );

    pos_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/current_position", 10,
        std::bind(&PathPlanner::pos_callback, this, std::placeholders::_1)
    );

    goal_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/navigation/goal", 10,
        std::bind(&PathPlanner::goal_callback, this, std::placeholders::_1)
    );

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/navigation/path", 5) 
    );
}

void PathPlanner::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    last_map_state_ = msg;
}

void PathPlanner::pos_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    last_pos_state_ = msg;
}

void PathPlanner::goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    last_goal_state_ = msg;
}

}  // namespace isc_snav

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto lp_node = std::make_shared<PathPlanner::PathPlanner>(options);
  exec.add_node(lp_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}