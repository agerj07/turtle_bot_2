#include "ros/ros.h"
#include "std_msgs/String.h"

void writeMsgToLog(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("The message that we received was: %s", msg->data.c_str());

}

int main(int argc, char *argv[]){



	ros::init(argc, argv, "Subscriber");
	ros::NodeHandle nh;

	ros::Subscriber topic_sub = nh.subscribe("tutorial", 1000, writeMsgToLog); //final entry is function to handle incoming message
	
	ros::spin();	//loops and waits for incoming messages



	return 0;



}
