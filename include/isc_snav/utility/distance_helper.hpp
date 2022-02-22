#define EPSILON 1e-3

// distance formula function that finds distance between two points
double distanceFormula( const Point3D& point1, const Point3D& point2 )
{
	return std::sqrt( std::pow( ( point2.x - point1.x ), 2 )
		+ std::pow( ( point2.y - point1.y ), 2 ) );
}

double distanceFormula( const Point2D& point1,
								 const Point2D& point2 )  // function overloading so usable with point2d
{
		return std::sqrt( std::pow( ( point2.x - point1.x ), 2 )
											+ std::pow( ( point2.y - point1.y ), 2 ) );
}

double distanceFormula(const Point2D& point1, const Point3D& point2 )  // function overloading so usable with point2d and path
{
		return std::sqrt( std::pow( ( point2.x - point1.x ), 2 )
											+ std::pow( ( point2.y - point1.y ), 2 ) );
}

double distanceFormula(const Point3D& point1, const Point2D& point2 )  // function overloading so usable with point2d and path
{
		return std::sqrt( std::pow( ( point2.x - point1.x ), 2 )
			+ std::pow( ( point2.y - point1.y ), 2 ) );
}

double ang_diff( double th1, double th2 )  // angular error
{
		return fmod( ( ( th1 - th2 ) + 3.0f * M_PI ), ( 2.0f * M_PI ) ) - M_PI;
}

bool approximately_equals( Point3D expected, Point3D actual )
{
		return ( std::fabs( actual.x - expected.x ) < EPSILON )
				&& ( std::fabs( actual.y - expected.y ) < EPSILON )
				&& ( std::fabs( actual.z - expected.z ) < EPSILON );
}

bool approximately_equals( Point2D expected, Point2D actual )
{
		return ( std::fabs( actual.x - expected.x ) < EPSILON )
				&& ( std::fabs( actual.y - expected.y ) < EPSILON );
}
