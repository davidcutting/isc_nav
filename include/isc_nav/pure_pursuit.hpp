// MIT License
//
// Copyright (c) Intelligent Systems Club
// Copyright (c) Avery Girven
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

#pragma once

#include <tuple>
#include <vector>
#include "utility/point.hpp"

typedef std::vector<Point3D> Path;
typedef std::pair<Point3D, Point3D> Segment3D;
typedef std::pair<Point2D, Point2D> Segment2D;

namespace PurePursuit
{
class PurePursuit 
{
public:
    PurePursuit(const Path& robot_path, const double& lookahead_distance);
    
    /**
    * @brief Find the target state (point and velocity) of the robot
    * @param state a Point3D where x and y are the position of the bot and z is heading
    * @return return Point3D that is lookahead point, first double is heading to point,
    * second double is the heading error
    */
    std::tuple<Point3D, double, double> get_target_state( const Point3D& state );

    /**
    * @brief will reset the path the robot must follow to a new robot path
    * @param new path for the robot to follow as robot_path
    */
    void reset_path( const Path& robot_path );

    /**
    * @brief will reset lookahead distance
    * @param the new lookahead distance
    */
    void reset_lookahead_distance( const double& lookahead_distance );

protected:
    std::pair<Point2D, double> project_to_line_segment( Point2D p, Segment2D seg );
    
    // function to scale values in one range to values in another range -  proportional
    // scaling.. this is to find z value
    double pscale( const double& x, const double& in_min, const double& in_max,
							 const double& out_min, const double& out_max );
    /**
    * @brief Will get the coordinates and target velocity of the lookahead point
    * @param the current location of the robot
    * @return Point3D variable for a point lookahead distance ahead of the state
    */
    Point3D get_lookahead_point( const Point3D& state );

    /**
    * @brief Will get the point on the path that corresponds to the distance form
    * beginning of path
     * @param position double value to represent point position
     * @return the point3D that is on the path
    */
    Point3D get_point_on_path( const double& position );

    /**
    * @brief will get the robots location on a path segment relevant to the location of
    * the robot
     * @param point of the robot (its current location_
     * @return  first value is the x,y location coordinate of robot corresponding with
    * the path. and the second value is a double distance between robot and the path
    */
    std::pair<Point2D, double> get_location_on_path( const Point2D& state );

    /**
     * @brief will find total length of robot path
     * @return total distance of path from first point to last point
     */
    double path_length();

    /**
     * @brief gets total distance from first point on path to point on path
     * @param point2D that exists on the path
     * @return distance double from first point on Path to the parameter point
     */
    double get_distance_to_point( const Point2D& currPoint );

    std::pair<Point2D, double> get_current_segment_location_on_path( const Point2D& state );

    Path m_robot_path;
    double m_lookahead_distance;
    uint32_t m_current_segment;
};
} // namespace pure pursuit
