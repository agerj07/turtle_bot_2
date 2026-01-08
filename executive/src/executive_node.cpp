#include "ros/ros.h"
#include "executive/executive.h"

int main(int argc, char* argv[]) {

	Executive executive;
	ros::init(argc, argv, "executive_node");
	ros::NodeHandle node_handle;
	ros::Subscriber subscriber_way_points = node_handle.subscribe( "waypoints", 1, &Executive::handleWayPoints, &executive );
	ros::Subscriber subscriber_estimated_odom_exec = node_handle.subscribe( "estimated_odom", 1, &Executive::handleEstimatedOdom, &executive );
	ros::Subscriber subscriber_odom_exec = node_handle.subscribe( "odom", 1, &Executive::handleOdom, &executive );
	
	ros::Publisher goal_publisher_exec = node_handle.advertise< geometry_msgs::Point32 >( "goal", 1, true );


	sleep(2);
	ros::Rate timer(10); // 10 Hz
	while( ros::ok() ){

		executive.wayPointsDealer();
		goal_publisher_exec.publish(executive._goal);
		ros::spinOnce();
		timer.sleep();

	}


	return 0;
}
