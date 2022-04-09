#pragma once

#include <isc_nav/utility/map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <stack>
#include <unordered_map>

namespace isc_nav
{
class BreadthFirstSearch
{
public:
    explicit BreadthFirstSearch(const nav_msgs::msg::OccupancyGrid& grid) : start_{}, goal_{}, map_(grid) {}

    const nav_msgs::msg::Path get_path(const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& goal_pose) noexcept
    {
        nav_msgs::msg::Path path;
        std::unordered_map<Point2D, Point2D> came_from;
        std::stack<Point2D> open_list_;
        start_ = Point2D(start_pose.position.x, start_pose.position.y);
        goal_ = Point2D(goal_pose.position.x, goal_pose.position.y);

        open_list_.emplace(start_);
        came_from[start_] = start_;

        while(!open_list_.empty())
        {
            // get the next position from the stack
            const auto& current = open_list_.top();
            open_list_.pop();

            // TODO: Param
            if ((current - goal_).x < 0.1 && (current - goal_).y < 0.1)
            {
                //std::cout << "Goal found, getting path." << std::endl;
                trace_back_path(current, path, came_from);
            }

            // add neighbors to list and build graph
            for (auto next : map_.neighbors(current))
            {
                // Don't add it to the open list if we already visited.
                if (came_from.find(next) != came_from.end())
                {
                    continue;
                }

                open_list_.emplace(next);
                came_from[next] = current;
            }
        }

        return path;
    }

    void trace_back_path(const Point2D& current, nav_msgs::msg::Path& path, std::unordered_map<Point2D, Point2D>& came_from) noexcept
    {
        auto parent = came_from[current];
        while (parent != start_)
        {
            geometry_msgs::msg::PoseStamped pose{};
            pose.pose.position.x = parent.x;
            pose.pose.position.y = parent.y;
            path.poses.emplace_back(pose);
            parent = came_from[parent];
        }
        std::reverse(path.poses.begin(), path.poses.end());
    }

private:
    Point2D start_;
    Point2D goal_;
    CostMap map_;
};
}