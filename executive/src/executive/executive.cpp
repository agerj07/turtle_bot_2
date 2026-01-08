#include "executive/executive.h"
#include <iostream>
#include <cmath>

Executive::
Executive(){
	wayPoint_counter = 0;
}

Executive::
~Executive(){}

void 
Executive::
handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	_estimated_odom = *msg;
	return;
}

void 
Executive::
handleOdom( const nav_msgs::Odometry::ConstPtr& msg){
	_odom = *msg;
	return;
}

void 
Executive::
handleWayPoints(const geometry_msgs::Polygon::ConstPtr& msg ){
	_wayPoints = *msg;
	return;	
}

void 
Executive::
wayPointsDealer(){
	
	if (_wayPoints.points.empty()) {
		std::cout << "No waypoints" << std::endl;
		return;
	}
	
	//float robotX = _estimated_odom.pose.pose.position.x;
	//float robotY = _estimated_odom.pose.pose.position.y;
	float robotX = _estimated_odom.pose.pose.position.x;
	float robotY = _estimated_odom.pose.pose.position.y;
	float wayPointX = _wayPoints.points[wayPoint_counter].x;
	float wayPointY = _wayPoints.points[wayPoint_counter].y;
	float threshold = _wayPoints.points[wayPoint_counter].z;
	std::cout << "Exec goalX: " << wayPointX << std::endl;
	std::cout << "Exec goalY: " << wayPointY << std::endl;
	if(	wayPoint_counter == 0){
		_goal = _wayPoints.points[0];
	}
	std::cout << "Threshold: " << threshold << std::endl;	
	if( sqrt( pow(wayPointX-robotX, 2) + pow(wayPointY-robotY, 2) ) < threshold){
		std::cout << "Robot distance to goal: " << sqrt( pow(wayPointX-robotX, 2) + pow(wayPointY-robotY, 2)) << std::endl;
		std::cout << "Goal Threshold: " << threshold << std::endl;
		if(wayPoint_counter < _wayPoints.points.size() - 1 ){
			wayPoint_counter++;
			_goal = _wayPoints.points[wayPoint_counter];
		}
	}

	return;
}
