#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[]){

	ros::init(argc, argv, "publisher");
	ros::NodeHandle node_handle;
	
	ros::Publisher topic_pub = node_handle.advertise<std_msgs::String>("tutorial", 1);
	ros::Rate loop_rate(1);
	
	while(ros::ok()){
		std_msgs::String msg;
		msg.data = "Hello World!";
	
		topic_pub.publish(msg);
		ros::spinOnce();	//checks for any callbacks
		loop_rate.sleep();	
	
	}

	return 0;


}

