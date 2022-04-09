#pragma once

#include <tuple>
#include <vector>

namespace isc_nav
{
/**
 * @brief data structure to represent x and y coordinates for the robot
 */
struct Point2D
{
	double x, y;
	Point2D( const double& ix, const double& iy )
		: x( ix )
		, y( iy )
	{
	}
	Point2D() {}
};

inline bool operator==( const Point2D& lhs, const Point2D& rhs )
{
  return ( lhs.x == rhs.x && lhs.y == rhs.y );
}
inline bool operator!=( const Point2D& lhs, const Point2D& rhs )
{
    return ( lhs.x != rhs.x && lhs.y != rhs.y );
}
inline Point2D operator-( const Point2D& lhs, const Point2D& rhs )
{
    return Point2D(lhs.x - rhs.x, lhs.y - rhs.y);
}

/**
 * @brief: data structure to present x, y, and z (velocity) of the robot
 */
struct Point3D
{
	double x, y, z;
	Point3D( const double& ix, const double& iy, const double& iz )
		: x( ix )
		, y( iy )
		, z( iz )
	{
	}
	Point3D() {}

	Point2D to2D() { return Point2D( x, y ); }
};

inline bool operator==(const Point3D& lhs, const Point3D& rhs)
{
  return ( lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z );
}
inline bool operator!=(const Point3D& lhs, const Point3D& rhs)
{
    return ( lhs.x != rhs.x && lhs.y != rhs.y && lhs.z != rhs.z );
}
}

namespace std
{
template<> struct hash<isc_nav::Point2D>
{
    [[nodiscard]] std::size_t operator()(const isc_nav::Point2D& point) const noexcept
    {
        return std::hash<size_t>()(static_cast<size_t>(point.x) ^ (static_cast<size_t>(point.y) << 16));
    }
};
} // namespace std

