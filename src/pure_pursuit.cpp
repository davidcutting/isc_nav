// MIT License
//
// Copyright (c) Intelligent Systems Club
// Copyright (c) Aaron Cofield, JessRose Narsinghia, Andrew R. Davis
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

#include "isc_snav/pure_pursuit.hpp"
#include "isc_snav/utility/distance_helper.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace PurePursuit
{

PurePursuit::PurePursuit(rclcpp::NodeOptions options) : Node("pure_pursuit", options)
{
  path_subscription = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&PurePursuit::path_callback, this, std::placeholders::_1));
   
  speed = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);
}

std::pair<Point2D, double> PurePursuit::project_to_line_segment( Point2D p, Segment2D seg )
{
	/* This implementation is a slightly modified version of the function from :
	 http://forums.codeguru.com/showthread.php?194400-Distance-between-point-and-line-segment
	*/
	double cx = p.x;
	double cy = p.y;
	double ax = seg.first.x;
	double ay = seg.first.y;
	double bx = seg.second.x;
	double by = seg.second.y;
	double r_numerator = ( cx - ax ) * ( bx - ax ) + ( cy - ay ) * ( by - ay );
	double r_denomenator = ( bx - ax ) * ( bx - ax ) + ( by - ay ) * ( by - ay );
	double r = r_numerator / r_denomenator;

	double px = ax + r * ( bx - ax );
	double py = ay + r * ( by - ay );

	double s = ( ( ay - cy ) * ( bx - ax ) - ( ax - cx ) * ( by - ay ) ) / r_denomenator;

	double distanceLine = fabs( s ) * sqrt( r_denomenator );
	double distanceSegment = -1;
	//
	// (xx,yy) is the point on the lineSegment closest to (cx,cy)
	//
	double xx = px;
	double yy = py;

	if ( ( r >= 0 ) && ( r <= 1 ) )
	{
		distanceSegment = distanceLine;
	}
	else
	{
		double dist1 = ( cx - ax ) * ( cx - ax ) + ( cy - ay ) * ( cy - ay );
		double dist2 = ( cx - bx ) * ( cx - bx ) + ( cy - by ) * ( cy - by );
		if ( dist1 < dist2 )
		{
			xx = ax;
			yy = ay;
			distanceSegment = sqrt( dist1 );
		}
		else
		{
			xx = bx;
			yy = by;
			distanceSegment = sqrt( dist2 );
		}
	}

	return std::make_pair( Point2D( xx, yy ), distanceSegment );
}

void PurePursuit::path_callback(const nav_msgs::msg::Path::SharedPtr path)
{
	std::cout << " Here lmao " << std::endl;
}

std::tuple<Point3D, double, double> PurePursuit::get_target_state( const Point3D& state )
{
	Point3D lookaheadTarget = get_lookahead_point( state );
	double headingTo = atan2( lookaheadTarget.y - state.y,
															lookaheadTarget.x - state.x );  // heading to point
															
	double headingErr = ang_diff( headingTo, state.z );       // heading error
	return std::make_tuple( lookaheadTarget, headingTo, headingErr );
}

double PurePursuit::path_length()
{
	double totalLength = 0;

	for ( int i = 0; i < ( m_robot_path.size() - 1 ); i++ )  // will use current point (i) and next point (i+1)
	{            																						// until i = the second to last point in vector
		totalLength += distanceFormula( m_robot_path.at( i ), m_robot_path.at( i + 1 ) );
	}

		return totalLength;
}

// function to scale values in one range to values in another range -  proportional
// scaling.. this is to find z value
double PurePursuit::pscale( const double& x, const double& in_min, const double& in_max,
							 const double& out_min, const double& out_max )
{
		return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;
}

void PurePursuit::reset_path( const Path& robot_path ) 
{ 
	m_robot_path = robot_path; 
}

void PurePursuit::reset_lookahead_distance( const double& lookahead_distance )
{
	m_lookahead_distance = lookahead_distance;
}

Point3D PurePursuit::get_lookahead_point( const Point3D& state )
{
	// use get_location on path with state and then use get distance from point, add five
	// to the return and call get point
	Point2D pointOnPath = get_location_on_path( { state.x, state.y } ).first;
//    Point2D pointOnPath = get_current_segment_location_on_path( { state.x, state.y } ).first;
	double dist_to_point = get_distance_to_point( pointOnPath );
	double lookaheadPointDistance = dist_to_point + m_lookahead_distance;
//    if(m_current_segment < m_robot_path.size() && lookaheadPointDistance > distanceFormula(m_robot_path.at(m_current_segment),m_robot_path.at(m_current_segment + 1))){
//        m_current_segment++;
//    }

	auto ret = get_point_on_path( lookaheadPointDistance );
//    std::cout << get_distance_to_point( pointOnPath ) << std::endl;
	return ret;
}

Point3D PurePursuit::get_point_on_path( const double& position )
{
	// constructor of this class initializes the path as m_robot_path (type Path (vector)
	// )
	// use the vector to find equation of a line and then from vect at 0
	// this function simply returns the x,y, and z coordinate that represents the path
	// blank position points from the beginning of the vector
	double numerator, denominator;
	double sum = 0;  // must be initialized to the first path point bc for loop adds on
	// the following point
	double distance, slope = 0;
	double zVal = 0, yVal = 0, xVal = 0;
	unsigned long vectSize = m_robot_path.size();
	Point3D newPoint;

	if ( ( path_length() > position ) && ( position > 0 ) )
	{
		for ( unsigned long i = 0; i < ( vectSize - 1 ); i++ )
		{  // loop until second to last point in path
			numerator = m_robot_path.at( i + 1 ).y
			- m_robot_path.at( i ).y;  // i + 1 looks ahead to next point in path in
		// order to act as point 2 in the slope
		// formula
		denominator = m_robot_path.at( i + 1 ).x - m_robot_path.at(i).x;  // denominator subtracts x values of two points

		if ( denominator != 0 )  // this means that line is not parallel to the y
		// axis
		{
			sum += distanceFormula( m_robot_path.at( i ),m_robot_path.at( i + 1 ) );  // distance formula
			if ( sum >= position )  // if the distance is greater than that means the
			// position is between i and i+1
			{
				distance = sum - distanceFormula( m_robot_path.at( i ), m_robot_path.at( i + 1 ) );
				distance = position - distance;
				slope = numerator / denominator;

				// in order to find x value of new point must use equation of a
				// circle with radius of distance
				// and center point of m_robot_path.at(i)... then the eq x = x1 +
				// distance/ (sgrt(1 + m^2) is derived
				if ( m_robot_path.at( i ).x < m_robot_path.at( i + 1 ).x )
				{
					xVal = m_robot_path.at( i ).x
						+ ( distance / std::sqrt( 1 + ( slope * slope ) ) );
				}
				else
				{
					xVal = m_robot_path.at( i ).x
						- ( distance / std::sqrt( 1 + ( slope * slope ) ) );
				}

				yVal = ( slope * xVal ) - ( slope * m_robot_path.at( i ).x )
					+ m_robot_path.at( i ).y;
				// FIND Z
				zVal = pscale( distance, 0,
					distanceFormula( m_robot_path.at( i ),
					m_robot_path.at( i + 1 ) ),
					m_robot_path.at( i ).z, m_robot_path.at( i + 1 ).z );
					break;  // to break out of for loop
			}
					// else it does not do anything
		}
		else  // line is parallel to y-axis
		{
			sum += distanceFormula( m_robot_path.at( i ), m_robot_path.at( i + 1 ) );
			if ( sum >= position )  // if the distance is greater than that means the
			// position is between i and i+1
			{
				distance = sum - distanceFormula( m_robot_path.at( i ),
					m_robot_path.at( i + 1 ) );
				distance = position - distance;

				xVal = m_robot_path.at( i ).x;  // point only moved on y axis between i and i + 1
				yVal = distance
					+ m_robot_path.at( i ).y;  // add the remaining distance to push point up

				// FIND Z
					zVal = pscale( distance, 0,
					distanceFormula( m_robot_path.at( i ),
					m_robot_path.at( i + 1 ) ),
					m_robot_path.at( i ).z, m_robot_path.at( i + 1 ).z );

					break;  // to break out of for loop
			}
		}
		}
	}
	else if ( position <= 0 )  // the postion is negative or 0 so return the first path location
	{
		xVal = m_robot_path.at( 0 ).x;
		yVal = m_robot_path.at( 0 ).y;
		zVal = m_robot_path.at( 0 ).z;
	}
	else  // the position is either larger than or equal to the path length
	{
		xVal = m_robot_path.at( vectSize - 1 ).x;
		yVal = m_robot_path.at( vectSize - 1 ).y;
		zVal = m_robot_path.at( vectSize - 1 ).z;
	}

	newPoint.x = xVal;
	newPoint.y = yVal;
	newPoint.z = zVal;

	return newPoint;
}

std::pair<Point2D, double> PurePursuit::get_current_segment_location_on_path( const Point2D& state )
{
	if(m_current_segment < m_robot_path.size()) {
		Segment2D curr_seg(m_robot_path.at(m_current_segment).to2D(), m_robot_path.at(m_current_segment + 1).to2D());
		return project_to_line_segment(state, curr_seg);
	}
	else
	{
		return std::make_pair<Point2D,double>(m_robot_path.back().to2D(), distanceFormula(state,m_robot_path.back().to2D()));
	}
}

std::pair<Point2D, double> PurePursuit::get_location_on_path( const Point2D& state )
{
	double shortest_dist = std::numeric_limits<double>::max();
	std::pair<Point2D, double> shortest_dist_point;
	shortest_dist_point.second = -1;

	if ( m_robot_path.size() < 2 )
	{
		throw std::runtime_error( "Path must contain at least 2 points" );
	}

	for ( unsigned long i = 0; i < m_robot_path.size() - 1; ++i )
	{
		auto line_proj = project_to_line_segment(
			state, std::make_pair( m_robot_path.at( i ).to2D(),
			m_robot_path.at( i + 1 ).to2D() ) );
		if ( line_proj.second < shortest_dist )
		{
			shortest_dist       = line_proj.second;
			shortest_dist_point = line_proj;
		}
	}
	if ( shortest_dist_point.second != -1 )
	{
		return shortest_dist_point;
	}
	else
	{
		throw std::runtime_error( "Unable to find shortest distance" );
	}
}

bool is_between( const Point2D& a, const Point2D& c, const Point2D& b )
{
	double crossproduct = ( c.y - a.y ) * ( b.x - a.x ) - ( c.x - a.x ) * ( b.y - a.y );

	// compare versus epsilon for floating point values, or != 0 if using integers
	if ( std::fabs( crossproduct ) > EPSILON )
	{
		return false;
	}

	double dotproduct = ( c.x - a.x ) * ( b.x - a.x ) + ( c.y - a.y ) * ( b.y - a.y );
	if ( dotproduct < 0 )
	{
		return false;
	}

	double squaredlengthba = ( b.x - a.x ) * ( b.x - a.x ) + ( b.y - a.y ) * ( b.y - a.y );
	if ( dotproduct > squaredlengthba )
	{
		return false;
	}

	return true;
}

double PurePursuit::get_distance_to_point(const Point2D& currPoint )  // assumption: the point 3D exists on the path
{
	double sum = 0;

	if ( approximately_equals( currPoint, m_robot_path.back().to2D() ) )
	{
		return path_length();
	}
	else if ( approximately_equals( currPoint, m_robot_path.front().to2D() ) )
	{
		return 0;
	}
	for ( unsigned long i = 0; i < m_robot_path.size() - 1; ++i )
	{
		bool b = is_between( m_robot_path.at( i ).to2D(), currPoint, m_robot_path.at( i + 1 ).to2D() );
		if ( !b )
		{
			sum += distanceFormula( m_robot_path.at( i ), m_robot_path.at( i + 1 ) );
		}
		else
		{
			sum += distanceFormula( m_robot_path.at( i ).to2D(), currPoint );
			break;
		}
	}
	if ( std::fabs( path_length() - sum ) < EPSILON )
	{
		return -1;
	}
	return sum;
}
} // namespace pure pursuit
