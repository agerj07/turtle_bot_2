#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "navigation/plannerxy.h"
#include "navigation/controller.h"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "control_publisher");
    ros::NodeHandle node_handle;

	Controller controller;

	ros::Subscriber subscriber_estimated_odom_control = node_handle.subscribe( "estimated_odom", 1, &Controller::handleEstimatedOdom, &controller );
	ros::Subscriber subscriber_path_control = node_handle.subscribe( "path", 1, &Controller::handlePath, &controller );
	ros::Subscriber subscriber_odom_control = node_handle.subscribe( "odom", 1, &Controller::handleOdom, &controller );

	//ros::Publisher command_publisher = node_handle.advertise< geometry_msgs::Twist >( "cmd_vel_mux/input/navi", 1, true );
	ros::Publisher command_publisher = node_handle.advertise< geometry_msgs::Twist >( "mobile base/commands/velocity", 1, true );
	ros::Publisher lookAhead_publisher = node_handle.advertise< geometry_msgs::PoseStamped >("look_ahead", 1, true);

	sleep(10);
	ros::Rate timer(30); // 30 Hz
	while (ros::ok()) {
        // Perform control calculations
		//std::cout << "controlCalc in pub" << std::endl; 
        //controller.controlCalc();
		command_publisher.publish(controller.move_control);
		lookAhead_publisher.publish(controller.lookAhead_pub);
  		ros::spinOnce();
		timer.sleep();

	}


}
