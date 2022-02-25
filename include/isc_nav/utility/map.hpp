#pragma once

#include <inttypes.h>
#include <isc_nav/utility/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

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

template<typename C>
class CostMap
{
public:
    CostMap(const uint32_t& width, const uint32_t& height, const float& resolution, const Point2D& origin)
        : size_x_{width}, size_y_{height}, resolution_{resolution}, origin_{origin}
    {
        costmap_.resize(size_x_ * size_y_);
    }

    CostMap(const nav_msgs::msg::OccupancyGrid& grid)
        : size_x_{grid.info.width}, size_y_{grid.info.height}, resolution_{grid.info.resolution}, costmap_(grid.data)
    {
        origin_ = Point2D(grid.info.origin.position.x, grid.info.origin.position.y);
    }

    C at(const Point2D& location) const noexcept
    {
        uint32_t x = std::static_cast<uint32_t>((location.x - origin_.x) / resolution_);
        uint32_t y = std::static_cast<uint32_t>((location.y - origin_.y) / resolution_);
        return at(x, y);
    }

    C at(const uint32_t& x, const uint32_t& y) const noexcept
    {
        return costmap_.at(y * size_x_ + x);
    }

    bool valid_cell(const uint32_t& x, const uint32_t& y) const noexcept
    {
        if (x > size_x_ || y > size_y_)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    std::vector<C> neighbors(const Point2D& current)
    {
        std::vector<C> neighbors{};
        uint32_t current_x = std::static_cast<uint32_t>((location.x - origin_.x) / resolution_);
        uint32_t current_y = std::static_cast<uint32_t>((location.y - origin_.y) / resolution_);
        
        // Check if a cell in each cardinal direction is valid. If so, put it into neighbor vector
        if (valid_cell(current_x    , current_y + 1)) neighbors.emplace_back( at(current_x    , current_y + 1) ); // top
        if (valid_cell(current_x + 1, current_y    )) neighbors.emplace_back( at(current_x + 1, current_y    ) ); // right
        if (valid_cell(current_x    , current_y - 1)) neighbors.emplace_back( at(current_x    , current_y - 1) ); // bottom
        if (valid_cell(current_x - 1, current_y    )) neighbors.emplace_back( at(current_x - 1, current_y    ) ); // left
        
        return neighbors;
    }

private:
    uint32_t size_x_;
    uint32_t size_y_;
    float resolution_;
    Point2D origin_;

    std::vector<C> costmap_;
};
} // namespace isc_nav
