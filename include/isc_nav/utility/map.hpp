#pragma once

#include <cstdint>
#include <inttypes.h>
#include "../utility/point.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "utility"

namespace isc_nav
{

/**
enum class CellType
{
    UNKNOWN,
    OBSTACLE,
    FREE,
};

class Cell
{
public:
    CellType type;
    uint8_t cost;
};
**/

class CostMap
{
public:
    CostMap(const uint32_t& width, const uint32_t& height, const float& resolution, const Point2D& origin)
        : size_x_{width}, size_y_{height}, resolution_{resolution}, origin_{origin}
    {
        costmap_.resize(size_x_ * size_y_);
    }

    CostMap(const nav_msgs::msg::OccupancyGrid& grid)
        : size_x_{grid.info.width}, size_y_{grid.info.height}, resolution_{grid.info.resolution}
    {
        costmap_.resize(size_x_ * size_y_);
        for (uint32_t i = 0; i < size_x_; i++)
        {
            for (uint32_t j = 0; j < size_y_; j++)
            {
                // convert occupancy grid to costmap
                costmap_[j * size_x_ + i] = occ_to_cost(grid.data[j * size_x_ + i]);
            }
        }
        origin_ = Point2D(grid.info.origin.position.x, grid.info.origin.position.y);
    }

    uint8_t occ_to_cost(const int8_t& occ_prob) const noexcept
    {
        if (occ_prob == OCC_GRID_UNKNOWN) return NO_INFORMATION;
        if (occ_prob == OCC_GRID_FREE) return FREE_SPACE;
        if (occ_prob == OCC_GRID_OCCUPIED) return LETHAL_OBSTACLE;
        // Linear transform from occ grid range to cost range
        constexpr double transform_ratio = (LETHAL_OBSTACLE - FREE_SPACE) / (OCC_GRID_OCCUPIED - OCC_GRID_FREE);
        return std::round(static_cast<double>(occ_prob) * transform_ratio);
    }

    uint8_t at(const Point2D& location) const noexcept
    {
        uint32_t x = static_cast<uint32_t>((location.x - origin_.x) / resolution_);
        uint32_t y = static_cast<uint32_t>((location.y - origin_.y) / resolution_);
        return at(x, y);
    }

    uint8_t at(const uint32_t& x, const uint32_t& y) const noexcept
    {
        return costmap_.at(y * size_x_ + x);
    }

    bool valid_cell(const int32_t& x, const int32_t& y) const noexcept
    {
        if (x > (int64_t)size_x_ || y > (int64_t)size_y_ ||
            x < 0       || y < 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    std::vector<Point2D> neighbors(const Point2D& current)
    {
        std::vector<Point2D> neighbors{};

        // transform floating point x,y into map indices
        int32_t current_x = static_cast<int32_t>((current.x - origin_.x) / resolution_);
        int32_t current_y = static_cast<int32_t>((current.y - origin_.y) / resolution_);

        int32_t top_x = current_x;
        int32_t top_y = current_y + 1;

        int32_t right_x = current_x + 1;
        int32_t right_y = current_y;

        int32_t bottom_x = current_x;
        int32_t bottom_y = current_y - 1;

        int32_t left_x = current_x - 1;
        int32_t left_y = current_y;

        // add neighbor to list if neighbor is a valid cell
        if (valid_cell(top_x, top_y))
        {
            // transform map indices back to floating point x,y
            double neigh_x = static_cast<double>(top_x * resolution_ + origin_.x);
            double neigh_y = static_cast<double>(top_y * resolution_ + origin_.y);
            neighbors.emplace_back(neigh_x, neigh_y);
        }
        if (valid_cell(right_x, right_y))
        {
            double neigh_x = static_cast<double>(right_x * resolution_ + origin_.x);
            double neigh_y = static_cast<double>(right_y * resolution_ + origin_.y);
            neighbors.emplace_back(neigh_x, neigh_y);
        }
        if (valid_cell(bottom_x, bottom_y))
        {
            double neigh_x = static_cast<double>(bottom_x * resolution_ + origin_.x);
            double neigh_y = static_cast<double>(bottom_y * resolution_ + origin_.y);
            neighbors.emplace_back(neigh_x, neigh_y);
        }
        if (valid_cell(left_x, left_y))
        {
            double neigh_x = static_cast<double>(left_x * resolution_ + origin_.x);
            double neigh_y = static_cast<double>(left_y * resolution_ + origin_.y);
            neighbors.emplace_back(neigh_x, neigh_y);
        }
        
        return neighbors;
    }

    double get_size_x() { return size_x_; }
    double get_size_y() { return size_y_; }
    float get_resolution() { return resolution_; }

    // Values cooresponding to definition of Occupancy Grid in ROS
    static constexpr int8_t OCC_GRID_UNKNOWN = -1;
    static constexpr int8_t OCC_GRID_FREE = 0;
    static constexpr int8_t OCC_GRID_OCCUPIED = 100;

    // Our definition of cost
    static constexpr uint8_t NO_INFORMATION = 255;
    static constexpr uint8_t LETHAL_OBSTACLE = 254;
    static constexpr uint8_t FREE_SPACE = 0;
    
private:
    uint32_t size_x_;
    uint32_t size_y_;
    float resolution_;
    Point2D origin_;

    std::vector<uint8_t> costmap_;
};
} // namespace isc_nav
