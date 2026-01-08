#include <Eigen/Dense>

#include "mapper/ocmap.h"

using namespace std;

bool
row_col_in_map( const nav_msgs::OccupancyGrid& map, const int& row, const int&
col ){
	if( ( row < 0 ) || ( col < 0 ) ){
		return false;
	} else if ( ( row >= map.info.width ) || ( col >= map.info.height ) ){
		return false;
	} else {
		return true;
	}
}

double
row_to_x( const nav_msgs::OccupancyGrid& map, const int& row, const double& discretization ){
	return ( discretization * ( double )( -( ( int )( map.info.width ) - 1 ) / 2.0 + row ) );
}

double
col_to_y( const nav_msgs::OccupancyGrid& map, const int& col, const double& discretization ){
	return ( discretization * ( double )( -( ( int )( map.info.height ) - 1 ) / 2.0 + col ) );
}

int
x_to_row( const nav_msgs::OccupancyGrid& map, const double& x, const double& discretization ){
	return round( x / discretization + ( double )( ( map.info.width - 1 ) / 2 ) );
}

int
y_to_col( const nav_msgs::OccupancyGrid& map, const double& y, const double& discretization ){
	return round( y / discretization + ( double )( ( map.info.height - 1 ) / 2 ) );
}

double
quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
	return atan2( 2.0 * ( quaternion.w * quaternion.z + quaternion.x * quaternion.y ), 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ) );
}

int8_t
double_to_int8( const double& arg ){
	int tmp = ( int )( round( arg * 20.0 ) );
	if( tmp < -127 ){
		tmp = -127;
	} else if ( tmp > 127 ){
		tmp = 127;
	}
	return ( int8_t )( tmp );
}

double
int8_to_double( const int8_t& arg ){
	return ( double )( arg ) * 0.05;
}

OCMap::
OCMap( const double& discretizationArg, const unsigned int& widthArg, const unsigned int& heightArg ) : ocmap(),
	discretization( discretizationArg ),
	xs( widthArg ),
	ys( heightArg ),
	l0( log( 0.5 / ( 1.0 - 0.5 ) ) ),
	locc( log( 0.95 / ( 1.0 - 0.95 ) ) ),
	lfree( log( 0.05 / ( 1.0 - 0.05 ) )) {

	ocmap.info.resolution = discretization;
	ocmap.info.width = widthArg;
	ocmap.info.height = heightArg;
	ocmap.info.origin.position.x = row_to_x( ocmap, 0, discretization );
	ocmap.info.origin.position.y = col_to_y( ocmap, 0, discretization );
	ocmap.data.resize( ocmap.info.width * ocmap.info.height, double_to_int8( l0 ) );

	for( unsigned int i = 0; i < ocmap.info.width; i++ ){
		xs[ i ] = row_to_x( ocmap, i, discretization );
	}

	for( unsigned int i = 0; i < ocmap.info.height; i++ ){
		ys[ i ] = col_to_y( ocmap, i, discretization );
	}
}

OCMap::
~OCMap() {}

void
OCMap::
handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg ){
	scan = *msg;
	return;
}

void
OCMap::
handleOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	odom = *msg;
	//updateGL();
	return;
}

bool
OCMap::
checkMap( const double& x, const double& y, const double& radius, const double& threshold ){
	int row = x_to_row( ocmap, x, discretization );
	int col = y_to_col( ocmap, y, discretization );
	int offset = ceil( radius / discretization );
	for( int i = -offset; i < offset; i++ ){
		double dx = row_to_x( ocmap, row + i, discretization ) - x;

		for( int j = -offset; j < offset; j++ ){
			double dy = col_to_y( ocmap, col + j, discretization ) - y;

			if( row_col_in_map( ocmap, row + i, col + j ) && ( sqrt( dx * dx + dy * dy ) < radius ) ){
				int index = ( row + i ) * ocmap.info.height + col + j;

				if( ocmap.data[ index ] > threshold ){
					return false;
				}
			}
		}
	}
	return true;
}

void
OCMap::
update( const geometry_msgs::Pose& pose, const sensor_msgs::LaserScan& scan ){
	double x = pose.position.x;
	double y = pose.position.y;
	double yaw = quaternion_to_yaw( pose.orientation );

	vector< int > occupied_cells;
	vector< int > free_cells;

	// search over all ranges
	for( unsigned int j = 0; j < scan.ranges.size(); j++ ){
		double scan_angle = scan.angle_min + j * scan.angle_increment;
		float offset = 0.1397;
		// check for occupied cells
		if( ( fabs( scan.ranges[ j ] - scan.range_max ) > discretization ) && ( scan.ranges[ j ] > discretization ) ){
			
			double scan_x = x + scan.ranges[ j ] * cos( yaw + scan_angle ) + ( offset*cos(yaw) ); //add offset for robot drive offset is 5.5 inches
			double scan_y = y + scan.ranges[ j ] * sin( yaw + scan_angle ) + ( offset*sin(yaw) );
			int row = x_to_row( ocmap, scan_x, discretization );
			int col = y_to_col( ocmap, scan_y, discretization );

			if( row_col_in_map( ocmap, row, col ) ){
				int index = row * ocmap.info.height + col;

				if( find( occupied_cells.begin(), occupied_cells.end(), index ) == occupied_cells.end() ){
					occupied_cells.push_back( index );
				}
			}
		}

		double scan_distance = 0.0;
		double scan_increment = discretization;

		// check for free cells
	while( scan_distance < scan.ranges[ j ] ){
		double scan_x = x + scan_distance * cos( yaw + scan_angle ) + ( offset*cos(yaw) );
		double scan_y = y + scan_distance * sin( yaw + scan_angle ) + ( offset*sin(yaw) );
		int row = x_to_row( ocmap, scan_x, discretization );
		int col = y_to_col( ocmap, scan_y, discretization );

		if( row_col_in_map( ocmap, row, col ) ){
			int index = row * ocmap.info.height + col;

			if( ( find( free_cells.begin(), free_cells.end(), index ) == free_cells.end() ) && ( find( occupied_cells.begin(), occupied_cells.end(), index ) == occupied_cells.end() ) ){
				free_cells.push_back( index );
			}
		}
		scan_distance += scan_increment;
	}
	}
// updating free _map log probability, left as an exercise for you, revisit the probabilistic robotics chapter on occupancy grid mapping for details about how to implement this
	for(int i = 0; i < free_cells.size(); i++){
		double ocmapval = int8_to_double(ocmap.data[free_cells[i]]);
		ocmap.data[free_cells[i]] = double_to_int8(ocmapval + lfree - l0);

	}
	

// updating occupied _map log probability, left as an exercise for you, revisit the probabilistic robotics chapter on occupancy grid mapping for details about how to implement this
	for(int j = 0; j < occupied_cells.size(); j++){
		double ocmap2 = int8_to_double(ocmap.data[occupied_cells[j]]);
		ocmap.data[occupied_cells[j]] = double_to_int8(ocmap2 + locc - l0);

	}

	
	return;
}

ostream&
operator<<( ostream& out, const OCMap& other ){
	return out;
}
