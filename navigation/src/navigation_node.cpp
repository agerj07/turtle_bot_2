#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "navigation/plannerxy.h"
#include "nav_msgs/Path.h"

int main(int argc, char* argv[]) {
    


    PlannerXY testPlan;
	
	ros::init(argc, argv, "navigation_node");
	ros::NodeHandle node_handle;
	ros::Subscriber subscriber_simulated_obstacles = node_handle.subscribe( "simulated_obstacles", 1, &PlannerXY::handleSimulatedObstacles, &testPlan );
	ros::Subscriber subscriber_estimated_odom = node_handle.subscribe( "estimated_odom", 1, &PlannerXY::handleEstimatedOdom, &testPlan );
	ros::Subscriber subscriber_odom_nav = node_handle.subscribe( "odom", 1, &PlannerXY::handleOdom, &testPlan );	
	ros::Subscriber subscriber_lookAhead_nav = node_handle.subscribe("look_ahead", 1, &PlannerXY::handleLookAhead, &testPlan);
	ros::Subscriber subscriber_goal_nav = node_handle.subscribe("goal", 1, &PlannerXY::handleGoal, &testPlan);

	ros::Publisher path_publisher_nav = node_handle.advertise<nav_msgs::Path>("path", 1, true);
	
	//make controller a seperate node
	
	sleep(2);
	ros::Rate timer(10); // 10 Hz
	while( ros::ok() ){
		testPlan.Solve();
    	// Convert your path deque to nav_msgs::Path message
		nav_msgs::Path path_msg;

		for (std::shared_ptr<NodeXY> node : testPlan.path) {
		    geometry_msgs::PoseStamped pose_stamped;
		    pose_stamped.pose.position.x = node->x;
		    pose_stamped.pose.position.y = node->y;
		    path_msg.poses.push_back(pose_stamped);
		}
    	path_publisher_nav.publish(path_msg);
		//goal_publisher.publish(goal);
		ros::spinOnce();
		timer.sleep();
	} 

	
    return 0;
}

