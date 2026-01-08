#include <QtWidgets/QApplication>
#include "gui/gui.h"

using namespace std;
int
main( int argc, char* argv[] ){
	QApplication app( argc, argv );
	ros::init( argc, argv, "gui" );
	ros::NodeHandle node_handle;
	GUI gui;
	ros::Subscriber subscriber_reset_odometry = node_handle.subscribe( "scan", 1, &GUI::handleLaserScan, &gui );
	ros::Subscriber subscriber_odom = node_handle.subscribe( "odom", 1, &GUI::handleOdom, &gui );
	ros::Subscriber subscriber_observations = node_handle.subscribe( "observations", 1, &GUI::handle_Observations, &gui);
	
	ros::Subscriber subscriber_estimated_odom = node_handle.subscribe( "estimated_odom", 1, &GUI::handleEstimatedOdom, &gui );
	ros::Subscriber subscriber_goal = node_handle.subscribe( "goal", 1, &GUI:: handleGoal, &gui );
	ros::Subscriber subscriber_landmarks = node_handle.subscribe( "landmarks", 1, &GUI::handleLandmarks, &gui );
	ros::Subscriber subscriber_observed_landmarks = node_handle.subscribe( "observed_landmarks", 1, &GUI::handleLandmarks, &gui );
	ros::Subscriber subscriber_scan = node_handle.subscribe( "scan", 1, &GUI::handleLaserScan, &gui );
	
	ros::Subscriber subscriber_map = node_handle.subscribe( "map", 1, &GUI::handleMap, &gui );
	ros::Subscriber subscriber_simulated_obstacles = node_handle.subscribe( "simulated_obstacles", 1, &GUI::handleSimulatedObstacles, &gui );
	ros::Subscriber subscriber_path = node_handle.subscribe( "path", 1, &GUI::handlePath, &gui);
	ros::Subscriber subscriber_lookAhead = node_handle.subscribe("look_ahead", 1, &GUI::handleLookAhead, &gui);	

	gui.show();
	return app.exec();
}

