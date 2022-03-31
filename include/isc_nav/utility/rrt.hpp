#pragma once

#include "isc_nav/utility/point.hpp"
#include <cmath>
#include <isc_nav/utility/map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>
#include <cstdlib>
#include <ctime>

namespace isc_nav
{
    class RRT 
    {
    public:
        explicit RRT(const nav_msgs::msg::OccupancyGrid& grid) : start_{}, goal_{}, map_(grid) {}

        const nav_msgs::msg::Path get_path(const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& goal_pose)
        {
            nav_msgs::msg::Path path;
            start_ = Point2D(start_pose.position.x, start_pose.position.y);
            goal_ = Point2D(goal_pose.position.x, goal_pose.position.y);

            Point2D random, Xnew, Xnearest;
            double Xnearest_dist = 0;

            int lim = 1000; // number of iterations

            // Graph with vertices and edges
            V.push_back(start_);
            E.push_back(0);
            G.push_back(E);

            for (int i = 0; i < lim; i++)
            {   
                random = randomPosition();

                // Find nearest vertex to the random node
                Xnearest = findNearest(V, random, Xnearest_dist);

                if (getDistance(Xnearest, random) <= max_dist_)
                {
                    // If random vertex is short
                    Xnew = Xnearest;
                }
                else
                {
                    Xnew = findNew(Xnearest, random, Xnearest_dist);
                }

                if (isInObstacle(Xnew))
                {
                    continue;
                }
                
                // Make sure no obstacle between Xnearest and Xnew
                if (!isCollisionFree(Xnearest, Xnew))
                {
                    continue;
                }

                // Get the index of to and from
                int attach_to = getIndex(V, Xnew);
                int attach_from = getIndex(V, Xnearest);

                // Add edge
                if (attach_to == -1)
                {
                    int add_new = V.size();
                    V.push_back(Xnew);
                    E.push_back(0);

                    for (int i = 0; i < attach_to; i++)
                    {
                        G.at(i).push_back(0);
                    }

                    G.at(attach_from).at(add_new)++;
                    G.push_back(E);
                }
                else 
                {
                    V.push_back(Xnew);
                    E.push_back(0);

                    for (int i = 0; i < attach_to; i++)
                    {
                        G.at(i).push_back(0);
                    }

                    G.at(attach_from).at(attach_to)++;
                    G.push_back(E);
                }

                // If vertex at goal return graph
                if ((Xnew.x - goal_.x < 0.1) && (Xnew.y - goal_.y < 0.1))
                {
                    trace_back_path(Xnew, path);
                }
            }

            return path;
        }


        Point2D randomPosition()
        {
            srand(time(0));

            while (true) 
            {
                int32_t x = fRand(0, map_.get_size_x());
                int32_t y = fRand(0, map_.get_size_y());

                if (map_.valid_cell(x, y)) 
                {
                    Point2D random_node(x, y);
                    return random_node;
                }
            }
        }


        Point2D findNearest(std::vector<Point2D>& V, Point2D random_node, double& Xnearest_dist)
        {
            double dist;
            Point2D Xnearest;

            if (!V.empty()) 
            {
                // Assign values to be compared
                Xnearest_dist = getDistance(V.at(0), random_node);
                Xnearest = V.at(0);
            }

            for (auto vertex : V)
            {
                dist = getDistance(vertex, random_node);
                if (dist < Xnearest_dist)
                {
                    Xnearest_dist = dist;
                    Xnearest = vertex;
                }
            }

            return Xnearest;
        }

        // p1 is Xnearest, p2 is random node
        Point2D findNew(Point2D& p1, Point2D& p2, double& Xnearest_dist)
        {
            Point2D Xnew;
            double n = max_dist_;
            double m = Xnearest_dist - n;

            Xnew.x = (m * p2.x + n * p1.x)/(m + n);
            Xnew.y = (m * p2.y + n * p1.y)/(m + n);

            return Xnew;
        }


        bool isInObstacle(Point2D& random_node)
        {
            return map_.at(random_node) == CostMap::LETHAL_OBSTACLE;
        }

        bool isCollisionFree(Point2D& p1, Point2D& p2)
        {
            float reso = map_.get_resolution();
            double dist = getDistance(p1, p2);
            int num_of_pt = int(std::ceil(dist / reso));
            Point2D pt = p1;

            for(int i = 0; i < num_of_pt; i++)
            {
                pt = generateNew(pt, p2, dist, reso);
                if (isInObstacle(pt))
                {
                    return false;
                }
            }
            
            return true;
        }

        Point2D generateNew(Point2D& p1, Point2D& p2, double& dist, float& reso)
        {
            Point2D Xnew;
            double n = reso;
            double m = dist - n;

            Xnew.x = (m * p2.x + n * p1.x)/(m + n);
            Xnew.y = (m * p2.y + n * p1.y)/(m + n);

            dist -= reso;
            return Xnew;
        }

        int32_t fRand(int32_t fMin, int32_t fMax)
        {
            int32_t f = (int32_t)rand() / RAND_MAX;
            return fMin + f * (fMax - fMin);
        }

        double getDistance(Point2D p1, Point2D p2)
        {
            double dist = std::sqrt(pow((p1.x + p2.x), 2) + pow((p1.y + p2.y), 2));
            return dist;
        }

        int getIndex(std::vector<Point2D> v, Point2D key)
        {
            auto it = find(v.begin(), v.end(), key);
        
            // If element was found
            if (it != v.end())
            {
                // Calculate the index for key
                int index = it - v.begin();
                return index;
            }
            else {
                // If the element is not present in the vector
                return -1;
            }
        }

        void trace_back_path(Point2D& current, nav_msgs::msg::Path& path) noexcept
        {
            int size = V.size();
            int attach_from = getIndex(V, current);

            while (current != start_)
            {
                geometry_msgs::msg::PoseStamped pose{};
                pose.pose.position.x = current.x;
                pose.pose.position.y = current.y;
                path.poses.emplace_back(pose);
                
                for (int i = 0; i < size; i++)
                {
                    if (G.at(i).at(attach_from))
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

        double max_dist_ = 50; // shld be shorter of longer??
    };

} // namespace isc_nav