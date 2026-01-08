#include <iostream>
#include "ros/ros.h"

#include "geometry_msgs/Polygon.h"

using namespace std;

int
main( int argc, char* argv[] ){

	geometry_msgs::Polygon obstacles;

	unsigned int num_obstacles = 50;
	double min_x = -10.0;
	double max_x = 10.0;
	double min_y = -10.0;
	double max_y = 10.0;
	double min_radius = 0.3;
	double max_radius = 0.7;

	for( unsigned int i = 0; i < num_obstacles; i++ ){
		obstacles.points.push_back( geometry_msgs::Point32() );
		obstacles.points.back().x = min_x + ( double )( rand() % 101 ) / ( 100.0 ) * ( max_x - min_x );
		obstacles.points.back().y = min_y + ( double )( rand() % 101 ) / ( 100.0 ) * ( max_y - min_y );
		obstacles.points.back().z = min_radius + ( double )( rand() % 101 ) / ( 100.0 ) * ( max_radius - min_radius );
	}
	 // Add the known obstacle centered at (1, 4) with a radius of 1
    geometry_msgs::Point32 known_obstacle_center;
    known_obstacle_center.x = 1.0;
    known_obstacle_center.y = 4.0;
    known_obstacle_center.z = 1.0; // radius

    obstacles.points.push_back(known_obstacle_center);
	
	ros::init( argc, argv, "obstacles_publisher_node" );
	ros::NodeHandle node_handle;
	ros::Publisher obstacles_publisher = node_handle.advertise< geometry_msgs::Polygon >( "obstacles", 1, true );

	obstacles_publisher.publish( obstacles );

	sleep( 1 );

	return 0;
	}

