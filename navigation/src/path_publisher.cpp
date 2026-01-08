#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "navigation/plannerxy.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle node_handle;

    ros::Publisher path_publisher = node_handle.advertise<nav_msgs::Path>("path", 1, true);
	//ros::Publisher goal_publisher = node_handle.advertise< geometry_msgs::Point32 >( "goal", 1, true );


    PlannerXY planner;
	geometry_msgs::Point32 goal;
	goal.x = 1.0;
	goal.y = 6.0;
	goal.z = 0.1;
    
    //planner.Solve(planner._estimated_odom, goal);

    
	sleep(2);
	ros::Rate timer(10); // 10 Hz
	while( ros::ok() ){
		planner.Solve();
    	// Convert path deque to nav_msgs::Path message
		nav_msgs::Path path_msg;

		for (std::shared_ptr<NodeXY> node : planner.path) {
		    geometry_msgs::PoseStamped pose_stamped;
		    pose_stamped.pose.position.x = node->x;
		    pose_stamped.pose.position.y = node->y;
		    path_msg.poses.push_back(pose_stamped);
		}
    	path_publisher.publish(path_msg);
		//goal_publisher.publish(goal);
		ros::spinOnce();
		timer.sleep();
	}
    

    return 0;
}

