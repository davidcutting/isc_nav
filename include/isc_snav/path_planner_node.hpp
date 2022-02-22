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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <memory>
#include <functional>
#include <nav_msgs/msg/detail/path__struct.hpp>

namespace isc_snav
{
class PathPlanner : public rclcpp::Node
{
public:
    explicit PathPlanner(rclcpp::NodeOptions options);

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_state_;
    geometry_msgs::msg::Pose::SharedPtr last_goal_state_;
    geometry_msgs::msg::Pose::SharedPtr last_pos_state_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void pos_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pos_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};
}  // namespace isc_snav
