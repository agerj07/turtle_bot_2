#include <iostream>
#include <cmath>
#include "simulator/simulator.h"
#include <cstdlib>
#include <ctime>

using namespace std;

/**
* x(0) = x position
* x(1) = y position
* x(2) = heading
* u(0) = linear velocity (v)
* u(1) = angular_velocity (w)
*/

geometry_msgs::Quaternion
yaw_to_quaternion( const double& yaw ){
	// your implementation here
	
	//yaw = atan2(r21, r11)
	/*
	double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

	*/

	//std::cout << "simulator yaw:" << yaw << std::endl;
	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos( yaw / 2.0 );
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin( yaw / 2.0 );
	return quaternion;
}

double sample(double b){
    double a = 0; 
    srand(static_cast<unsigned>(time(0)));

    for(int i = 0; i < 12; i++){
        
        a += (static_cast<double>(rand()) / RAND_MAX) * 2 * b - b;
    }
    
    return a;
}

Simulator::
Simulator() : _x( 0.0, 0.0, 0 ), _u( 0.0, 0.0, 0.0 ) {

	_num_scan_angles = 128;
}

Simulator::
~Simulator() {

}

void
Simulator::
step( const double& dt ){
	// your implementation here
	_x(0) += (_u(0) * cos(_x(2)) * dt);
	_x(1) += (_u(0) * sin(_x(2)) * dt);
	_x(2) += _u(1) * dt;
	//want to keep bearing between -pi and pi for sim
	while(_x(2) > M_PI){
		_x(2) -= 2 * M_PI;
	
	}
	while(_x(2) < (-1 * M_PI)){
		_x(2) += 2 * M_PI;
	}
			
	return;
}

void
Simulator::
handle_command( const geometry_msgs::Twist::ConstPtr& msg ){
	_u( 0 ) = msg->linear.x;
	_u( 1 ) = msg->angular.z;
	return;
}

nav_msgs::Odometry
Simulator::
odometry_msg( void )const{
	nav_msgs::Odometry msg;
	msg.header.stamp = ros::Time::now();
	msg.pose.pose.position.x = _x( 0 );
	msg.pose.pose.position.y = _x( 1 );
	msg.pose.pose.position.z = 0.0;
	msg.pose.pose.orientation = yaw_to_quaternion( _x( 2 ) );
	msg.twist.twist.linear.x = _u( 0 );
	msg.twist.twist.angular.z = _u( 1 );
	return msg;
}



void
Simulator::
handle_obstacles( const geometry_msgs::Polygon::ConstPtr& msg ){
	_obstacles = *msg;
	return;
}

void 
Simulator::
handle_landmarks( const perception::Landmarks::ConstPtr& msg ){
	_landmarks = *msg;
	return;
}

void 
Simulator::
update_observed_landmarks_and_observations( void ){
	_observed_landmarks.landmarks.clear();	
	for(int i = 0; i < _landmarks.landmarks.size(); i++){
		double mx = _landmarks.landmarks[i].x;
		double my = _landmarks.landmarks[i].y;
		//update observed landmarks
		double range = sqrt(pow( (mx - _x(0)), 2) + pow( (mx - _x(0)), 2) );
		double bearing = atan2(my - _x(1), mx - _x(0)) - _x(2);

				
		if( (range > 0.1) && (range < 5.0) && (bearing > -M_PI/4.0) && (bearing < M_PI/4.0) ){	//check to see if in the range	
			//std::cout << "Observed landmark spotted" << std::endl;
			_observed_landmarks.landmarks.push_back( perception::Landmark() );			
			_observed_landmarks.landmarks.back().x = mx;
			_observed_landmarks.landmarks.back().y = my;
			_observed_landmarks.landmarks.back().signature = _landmarks.landmarks[i].signature;
		}


		//_observed_landmarks.landmarks[i].range = sqrt((mx - _x(0)) * (mx - _x(0)) + ((my - _x(1)) * (my - _x(1)));
		//_observed_landmarks.landmarks[i].bearing = atan2(my - _x(1), mx - _x(0)) - _x(2);
		//_observed_landmarks.landmarks[i].signature = _landmarks.landmarks[i].signature;
	}
	//update the observations
	_observations.observations.clear();
	for(int j = 0; j < _observed_landmarks.landmarks.size(); j++){
		double mx2 = _observed_landmarks.landmarks[j].x;
		double my2 = _observed_landmarks.landmarks[j].y;
		//std::cout << "Observation data updated" << std::endl;
		_observations.observations.push_back( perception::Observation() );		
		_observations.observations.back().range = sqrt((mx2 - _x(0)) * (mx2 - _x(0)) + (my2 - _x(1)) * (my2 - _x(1))) + sample(0.001);
		_observations.observations.back().bearing = atan2(my2 - _x(1), mx2 - _x(0)) - _x(2) + sample(0.001);
		_observations.observations.back().signature = _observed_landmarks.landmarks[j].signature;
	}
	
	return;
}
/*
void 
Simulator::
update_observations( void ){
	//update the observations using the observations msg and add noise
		
	//observations.observations.clear();	
	for (int i = 0; i < _observations.observations.size(); i++){
		double bearing = _observations.observations[i].bearing;
        double range = _observations.observations[i].range;
		double yaw = yaw_to_quarternion(_x(2));        
		//observed position relative to the robot
        double dx = range * cos(bearing + yaw);
        double dy = range * sin(bearing + yaw);
        //observed position based on robot position
        double updated_x = _x(0) + dx;
        double updated_y = _x(1) + dy;
        //updated range and bearing
        double updated_range = sqrt(updated_x * updated_x + updated_y * updated_y);
        double updated_bearing = atan2(updated_y, updated_x);   
        _observations.observations[i].range = updated_range + sample(0.001);
        _observations.observations[i].bearing = updated_bearing + sample(0.001);	
	
	}
}
*/
sensor_msgs::LaserScan
Simulator::
scan_msg( void )const{
	sensor_msgs::LaserScan msg;
	msg.angle_min = -M_PI/4.0;
	msg.angle_max = M_PI/4.0;
	msg.angle_increment = ( msg.angle_max - msg.angle_min ) / ( double )(_num_scan_angles - 1 );
	msg.range_min = 0.1;
	msg.range_max = 5.0;

	vector< double > scan_angles( _num_scan_angles );
	for( unsigned int i = 0; i < _num_scan_angles; i++ ){
		scan_angles[i] = msg.angle_min + ( double )( i ) * msg.angle_increment;
	}
	
	vector< double > obstacle_angles( _obstacles.points.size() );
	vector< double > obstacle_distances( _obstacles.points.size() );
	vector< double > obstacle_phimaxs( _obstacles.points.size() );
	
	for( unsigned int i = 0; i < _obstacles.points.size(); i++ ){
		obstacle_angles[ i ] = atan2( _obstacles.points[ i ].y - _x( 1 ), _obstacles.points[ i ].x - _x( 0 ) ) - _x( 2 );
		if( obstacle_angles[ i ] < -M_PI ){
			obstacle_angles[ i ] += 2.0 * M_PI;
		} else if ( obstacle_angles[ i ] > M_PI ){
			obstacle_angles[ i ] -= 2.0 * M_PI;
		}
		obstacle_distances[ i ] = sqrt( pow( _obstacles.points[ i ].x - _x( 0 ), 2.0 ) + pow( _obstacles.points[ i ].y - _x( 1 ), 2.0 ) );
		// check to make sure we are not inside of an obstacle
		if( obstacle_distances[ i ] < _obstacles.points[ i ].z ){
			return msg;
		}
		obstacle_phimaxs[ i ] = atan2( _obstacles.points[ i ].z, sqrt( pow( obstacle_distances[ i ], 2.0 ) - pow( _obstacles.points[ i ].z, 2.0 ) ) );
	}
	
	for( unsigned int i = 0; i < scan_angles.size(); i++ ){
		double min_range = msg.range_max;

		for( unsigned int j = 0; j < _obstacles.points.size(); j++ ){
			// check to see if center of obstacle is within sensor range
			if( obstacle_distances[ j ] < ( msg.range_max + _obstacles.points[ j ].z) ){
			// check angle to see if in range range
				if( ( scan_angles[ i ] > ( obstacle_angles[ j ] - obstacle_phimaxs[ j ] ) ) && ( scan_angles[ i ] < ( obstacle_angles[ j ] + obstacle_phimaxs[ j ] ) ) ){
					double phi = scan_angles[ i ] - obstacle_angles[ j ];


					double a = 1.0 + pow( tan( phi ), 2.0 );
					double b = -2.0 * obstacle_distances[ j ];
					double c = pow( obstacle_distances[ j ], 2.0 ) - pow( _obstacles.points[ j ].z, 2.0 );
					
					// compute candidate intersection points
					double x1 = ( -b + sqrt( pow( b, 2.0 ) - 4 * a * c ) ) / ( 2.0 * a );
					double y1 = tan( phi ) * x1;
					double d1squared = pow( x1, 2.0 ) + pow( y1, 2.0 );

					double x2 = ( -b - sqrt( pow( b, 2.0 ) - 4 * a * c ) ) / ( 2.0 * a );
					double y2 = tan( phi ) * x2;
					double d2squared = pow( x2, 2.0 ) + pow( y2, 2.0 );

					double range = 0.0;
					if( d1squared < d2squared ){
						range = sqrt( d1squared );
					} else {
						range = sqrt( d2squared );
					}
					if( range < min_range ){
						min_range = range;
					}
				}
			}
		}
		if( min_range > msg.range_min ){
			msg.ranges.push_back( std::min( min_range + sample( 0.001 ), ( double )( msg.range_max ) ) );
		} else {
			msg.ranges.push_back( 0.0 );
		}
	}
}

geometry_msgs::Polygon
Simulator::
simulated_obstacles_msg( void )const{
	return _obstacles;
}


