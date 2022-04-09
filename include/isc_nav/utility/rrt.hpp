#pragma once

#include <isc_nav/utility/point.hpp>
#include <cmath>
#include <isc_nav/utility/map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <random>

namespace isc_nav
{
    class RRT 
    {
    public:
        /**
        * @brief Construct a new RRT object and seed the number generator
        * @param grid occupancy grid
        */
        //RRT();
        RRT(const nav_msgs::msg::OccupancyGrid& grid) : start_{}, goal_{}, map_(grid) {
            std::random_device rd;
            std::mt19937 rng(rd());
            
            this->rng = rng;
        }

        /**
         * @brief Get path to start rrt
         * @param start_pose a 2D point
         * @param goal_pose a 2D point
         * @return const nav_msgs::msg::Path 
         */
        const nav_msgs::msg::Path get_path(const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& goal_pose)
        {
            nav_msgs::msg::Path path;
            start_ = Point2D(start_pose.position.x, start_pose.position.y);
            goal_ = Point2D(goal_pose.position.x, goal_pose.position.y);

            Point2D random{}, Xnew{}, Xnearest{};
            double Xnearest_dist = 0;

            // Graph with vertices and edges
            V.push_back(start_);
            E.push_back(0);
            G.push_back(E);

            for (int i = 0; i < lim_; i++)
            {   
                random = randomPosition();

                Xnearest = findNearest(random, Xnearest_dist);

                if (getDistance(Xnearest, random) <= max_dist_)
                {
                    // If the random node is within the max_dist_ set it as the new node
                    Xnew = Xnearest;
                }
                else
                {   
                    Xnew = findNew(Xnearest, random, Xnearest_dist);
                }

                if (isInObstacle(Xnew))
                {
                    continue; // Skip this loop
                }
                
                if (!isCollisionFree(Xnearest, Xnew))
                {
                    continue; // Skip this loop
                }

                // Get the index of new random node and the nearest vertex
                int attach_to = getIndex(Xnew);
                int attach_from = getIndex(Xnearest);

                if (attach_from == -1) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Xnearest does not exist");
                    continue; // Skip this loop
                }

                if (attach_to == -1)
                {
                    int add_new = V.size();
                    V.push_back(Xnew);
                    E.push_back(0);

                    for (int i = 0; i < add_new; i++)
                    {
                        // Fill in the new column
                        G.at(i).push_back(0);
                    }

                    // Fill in new row
                    G.push_back(E);
                    // Add edge
                    G.at(attach_from).at(add_new)++;
                }
                else 
                {
                    // Add edge
                    G.at(attach_from).at(attach_to)++;
                }

                // If vertex at goal return graph
                if ((Xnew.x - goal_.x < 0.1) && (Xnew.y - goal_.y < 0.1))
                {
                    trace_back_path(Xnew, path);
                }
            }

            return path;
        }

        /**
         * @brief Generate a random position on the map
         * @return Point2D 
         */
        Point2D randomPosition()
        {   
            std::uniform_real_distribution<double> dist_x(0.0, map_.get_size_x());
            std::uniform_real_distribution<double> dist_y(0.0, map_.get_size_y());
            while (true) 
            {
                double x = dist_x(rng);
                double y = dist_y(rng);

                // Check if the generated random node is on the map
                if (map_.valid_cell(x, y)) 
                {
                    Point2D random_node(x, y);
                    return random_node;
                }
            }
        }

        /**
         * @brief Get the vertex (already generated) nearest to the random node
         * @param V a list of generated vertices
         * @param random_node 
         * @param Xnearest_dist track the distance between nearest vertex and random node
         * @return Point2D 
         */
        Point2D findNearest(const Point2D& random_node, double& Xnearest_dist)
        {
            double dist{};
            Point2D Xnearest{};

            if (!V.empty()) 
            {
                // Assign values to be compared
                Xnearest_dist = getDistance(V.at(0), random_node);
                Xnearest = V.at(0);
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Empty list");
                return Xnearest;
            }

            for (auto vertex : V)
            {
                dist = getDistance(vertex, random_node);
                if (dist < Xnearest_dist)
                {   
                    // Update distance between nearest vertex and random node
                    Xnearest_dist = dist;
                    Xnearest = vertex;
                }
            }

            return Xnearest;
        }

        /**
         * @brief Get a new node that is on the line (connecting the nearest vertex to random node) 
                    and within the max_dist_ to random node
         * @param p1 nearest vertex (already in the generated list)
         * @param p2 random node
         * @param Xnearest_dist track the distance between nearest vertex and random node
         * @return Point2D 
         */
        Point2D findNew(const Point2D& p1, const Point2D& p2, const double& Xnearest_dist)
        {
            Point2D Xnew{};
            double n = max_dist_;
            double m = Xnearest_dist - n;

            Xnew.x = (m * p2.x + n * p1.x)/(m + n);
            Xnew.y = (m * p2.y + n * p1.y)/(m + n);

            return Xnew;
        }

        /**
         * @brief Check if the new random node is at obstacle on map
         * @param random_node a random generated nose
         * @return true 
         * @return false 
         */
        bool isInObstacle(const Point2D& random_node)
        {
            return map_.at(random_node) == CostMap::LETHAL_OBSTACLE;
        }

        /**
         * @brief Check if there is any obstacle between the nearest vertex and random node
         * @param p1 nearest node
         * @param p2 random node (aka new node)
         * @return true 
         * @return false 
         */
        bool isCollisionFree(const Point2D& p1, const Point2D& p2)
        {
            float reso = map_.get_resolution();
            double dist = getDistance(p1, p2);
            int num_of_pt = static_cast<int>(dist / reso);
            Point2D pt = p1;

            for(int i = 0; i < num_of_pt; i++)
            {
                pt = generateAlongLine(pt, p2, dist, reso);
                if (isInObstacle(pt))
                {
                    return false;
                }
            }
            
            return true;
        }

        /**
         * @brief Generate a node on the line connecting nearest vertex and new random node
         * @param p1 new generated node along the line
         * @param p2 new random node
         * @param dist distance between the new generated along the line and the random node
         * @param reso resolution of the map
         * @return Point2D 
         */
        Point2D generateAlongLine(const Point2D& p1, const Point2D& p2, double& dist, const float& reso)
        {
            Point2D Xnew{};
            double n = reso;
            double m = dist - n;

            Xnew.x = (m * p2.x + n * p1.x)/(m + n);
            Xnew.y = (m * p2.y + n * p1.y)/(m + n);

            dist -= reso;
            return Xnew;
        }

        /**
         * @brief Get the distance between 2 points on map
         * @param p1 a 2D point
         * @param p2 a 2D point
         * @return double 
         */
        double getDistance(const Point2D& p1, const Point2D& p2)
        {
            double dist = std::sqrt(pow((p1.x + p2.x), 2) + pow((p1.y + p2.y), 2));
            return dist;
        }

        /**
         * @brief Get the index of a point in the vector list (which corresponds to graph G)
         * @param v 
         * @param key 
         * @return int 
         */
        int getIndex(const Point2D& key)
        {
            auto it = find(V.begin(), V.end(), key);
        
            // If element was found
            if (it != V.end())
            {
                // Calculate the index for key
                int index = it - V.begin();
                return index;
            }
            else {
                // If the element is not present in the vector
                return -1;
            }
        }

        /**
         * @brief Generate a path from the graph
         * @param current the latest generated random node that falls in the goal
         * @param path
         */
        void trace_back_path(const Point2D& end, nav_msgs::msg::Path& path) noexcept
        {
            int size = V.size();
            Point2D current = end;
            int curr_index = getIndex(current);

            while (current != start_)
            {
                geometry_msgs::msg::PoseStamped pose{};
                pose.pose.position.x = current.x;
                pose.pose.position.y = current.y;
                path.poses.emplace_back(pose);
                
                for (int i = 0; i < size; i++)
                {
                    if (G.at(i).at(curr_index))
                    {
                        current = V.at(i);
                        break;
                    }
                }
            }

            std::reverse(path.poses.begin(), path.poses.end());
        }

    private:
        Point2D start_;
        Point2D goal_;
        CostMap map_;

        std::vector<std::vector<int>> G;
        std::vector<int> E;
        std::vector<Point2D> V;

        // Constants
        double max_dist_ = 50; // Shld be shorter of longer??
        int lim_ = 1000; // Number of iterations

        // Random number generator
        std::mt19937 rng;
    };

} // namespace isc_nav