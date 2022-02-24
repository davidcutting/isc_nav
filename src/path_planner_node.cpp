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
#include <isc_nav/astar.hpp>

namespace isc_nav
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
        "/navigation/path", 5
    );

    // creates mock occupancy grid for testing
    for (int i = 0; i < 10; i++)
    {
        vector<int> new_row;
        for (int j = 0; j < 10; j++)
        {

            if (j % 3 == 0)
            {
                new_row.push_back(1);
            }
            else
            {
                new_row.push_back(0);
            }
        }

        occupancy_grid.push_back(new_row);
    }

    // creates the coordinates for the robot location and goal location
    robot_location.x = 9;
    robot_location.y = 9;

    goal_location.x = 0;
    goal_location.y = 0;

    // creates the node for the starting location
    node *start = new node;
    start->g_n = 0;
    start->h_n = findHn(robot_location, goal_location);
    start->path.push_back(robot_location);
    frontier.push(start);

    // starts searching for the solution
    expandNode();
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