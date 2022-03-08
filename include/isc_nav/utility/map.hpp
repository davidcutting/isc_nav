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

class CostMap
{
public:
    using CostMapIndex = std::pair<uint32_t, uint32_t>;

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

    CostMapIndex to_index(const Point2D& location) const noexcept
    {
        return CostMapIndex(static_cast<uint32_t>((location.x - origin_.x) / resolution_),  // x
                            static_cast<uint32_t>((location.y - origin_.y) / resolution_)); // y
    }

    Point2D to_point(const CostMapIndex& index) const noexcept
    {
        return Point2D(static_cast<double>(index.first * resolution_ + origin_.x),   // x
                       static_cast<double>(index.second * resolution_ + origin_.y)); // y
    }

    uint8_t at(const Point2D& location) const noexcept
    {
        return at(to_index(location));
    }

    uint8_t at(const CostMapIndex& index) const noexcept
    {
        return costmap_.at(index.second * size_x_ + index.first);
    }

    bool valid_cell(const int32_t& x, const int32_t& y) const noexcept
    {
        // not bigger than map size or negative
        return !(x > static_cast<int64_t>(size_x_) || y > static_cast<int64_t>(size_y_) ||
                 x < 0 || y < 0 ||
                 at(CostMapIndex(x, y)) != LETHAL_OBSTACLE); // TODO: Make args not x,y
    }

    std::vector<CostMapIndex> neighbors(const CostMapIndex& current)
    {
        std::vector<CostMapIndex> neighbors{};

        // transform floating point x,y into map indices
        int32_t top_x = current.first;
        int32_t top_y = current.second + 1;

        int32_t right_x = current.first + 1;
        int32_t right_y = current.second;

        int32_t bottom_x = current.first;
        int32_t bottom_y = current.second - 1;

        int32_t left_x = current.first - 1;
        int32_t left_y = current.second;

        if (valid_cell(top_x, top_y)) neighbors.emplace_back(top_x, top_y);
        if (valid_cell(right_x, right_y)) neighbors.emplace_back(right_x, right_y);
        if (valid_cell(bottom_x, bottom_y)) neighbors.emplace_back(bottom_x, bottom_y);
        if (valid_cell(left_x, left_y)) neighbors.emplace_back(left_x, left_y);
        
        return neighbors;
    }

private:
    uint32_t size_x_;
    uint32_t size_y_;
    float resolution_;
    Point2D origin_;

    std::vector<uint8_t> costmap_;

    // Values cooresponding to definition of Occupancy Grid in ROS
    static constexpr int8_t OCC_GRID_UNKNOWN = -1;
    static constexpr int8_t OCC_GRID_FREE = 0;
    static constexpr int8_t OCC_GRID_OCCUPIED = 100;

    // Our definition of cost
    static constexpr uint8_t NO_INFORMATION = 255;
    static constexpr uint8_t LETHAL_OBSTACLE = 254;
    static constexpr uint8_t FREE_SPACE = 0;
};
} // namespace isc_nav

namespace std
{
template<> struct hash<isc_nav::CostMap::CostMapIndex>
{
    [[nodiscard]] size_t operator()(const isc_nav::CostMap::CostMapIndex& point) const noexcept
    {
        return hash<size_t>()(static_cast<size_t>(point.first) ^ (static_cast<size_t>(point.second) << 16));
    }
};
} // namespace std