#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"

using namespace std;

int
main( int argc, char* argv[] ){

	geometry_msgs::Point32 waypoint1;
	waypoint1.x = 4.0;
	waypoint1.y = 5.0;
	waypoint1.z = 0.1;

	geometry_msgs::Point32 waypoint2;
	waypoint2.x = -3.0;
	waypoint2.y = 4.0;
	waypoint2.z = 0.2;

	geometry_msgs::Polygon waypoints;
	waypoints.points.push_back( waypoint1 );
	waypoints.points.push_back( waypoint2 );

	ros::init( argc, argv, "waypoints_publisher" );
	ros::NodeHandle node_handle;
	ros::Publisher waypoints_publisher = node_handle.advertise<geometry_msgs::Polygon >( "waypoints", 1, true );

	sleep(5);
	waypoints_publisher.publish( waypoints );
	sleep(1);

	return 0;
}

