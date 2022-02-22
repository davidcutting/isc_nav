#pragma once

#include "point.hpp"
#include <math.h>

constexpr double EPSILON = 1e-3;

// distance formula function that finds distance between two points
inline double distanceFormula( const Point3D& point1, const Point3D& point2 )
{
	return std::sqrt( std::pow( ( point2.x - point1.x ), 2 ) + std::pow( ( point2.y - point1.y ), 2 ) );
}

// function overloading so usable with point2d
inline double distanceFormula( const Point2D& point1, const Point2D& point2 )
{
	return std::sqrt( std::pow( ( point2.x - point1.x ), 2 ) + std::pow( ( point2.y - point1.y ), 2 ) );
}

// function overloading so usable with point2d and path
inline double distanceFormula(const Point2D& point1, const Point3D& point2 )
{
	return std::sqrt( std::pow( ( point2.x - point1.x ), 2 ) + std::pow( ( point2.y - point1.y ), 2 ) );
}

// function overloading so usable with point2d and path
inline double distanceFormula(const Point3D& point1, const Point2D& point2 )  
{
	return std::sqrt( std::pow( ( point2.x - point1.x ), 2 ) + std::pow( ( point2.y - point1.y ), 2 ) );
}

inline double ang_diff( double th1, double th2 )  // angular error
{
	return fmod( ( ( th1 - th2 ) + 3.0f * M_PI ), ( 2.0f * M_PI ) ) - M_PI;
}

inline bool approximately_equals(const Point3D& expected, const Point3D& actual )
{
	return ( std::fabs( actual.x - expected.x ) < EPSILON )
		&& ( std::fabs( actual.y - expected.y ) < EPSILON )
		&& ( std::fabs( actual.z - expected.z ) < EPSILON );
}

inline bool approximately_equals( const Point2D& expected, const Point2D& actual )
{
	return ( std::fabs( actual.x - expected.x ) < EPSILON )
		&& ( std::fabs( actual.y - expected.y ) < EPSILON );
}
