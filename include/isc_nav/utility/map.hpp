#pragma once

#include <inttypes.h>
#include <isc_nav/utility/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace isc_nav
{
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

template<typename C>
class CostMap
{
public:
    CostMap(const uint32_t& width, const uint32_t& height, const float& resolution);
    CostMap(const nav_msgs::msg::OccupancyGrid& grid);

    C at(const Point2D& location);
    C at(const uint32_t& x, const uint32_t& y);

    std::vector<C> neighbors(const Point2D& current);
private:
    uint32_t size_x_;
    uint32_t size_y_;
    float resolution_;
    Point2D origin_;

    std::vector<C> costmap_;
};
} // namespace isc_nav
