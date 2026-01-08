#include <iostream>
#include "ros/ros.h"
#include "mapper/ocmap.h"
using namespace std;

int main(int argc, char* argv[]){
	OCMap map;
	ros::init(argc, argv, "mapper_publisher");
	ros::NodeHandle node_handle;
	//subscribers
	ros::Subscriber subscriber_reset_odometry = node_handle.subscribe( "scan", 1, &OCMap::handleLaserScan, &map );
	ros::Subscriber subscriber_odom = node_handle.subscribe( "odom", 1, &OCMap::handleOdom, &map );
	
	

	ros::Publisher OccGrid = node_handle.advertise< nav_msgs::OccupancyGrid >("map", 1, true);

	sleep(2);
	double frequency = 10.0;
	ros::Rate timer( frequency );
	while( ros::ok() ){
		map.update(map.odom.pose.pose, map.scan);
		OccGrid.publish( map.ocmap );		
		ros::spinOnce();
		timer.sleep();
		
	}



	return 0;
}
