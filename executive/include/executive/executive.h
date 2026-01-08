#ifndef EXECUTIVE_H
#define EXECUTIVE_H

#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"

class Executive{
	public:
		Executive();
		virtual ~Executive();
		void handleWayPoints( const geometry_msgs::Polygon::ConstPtr& msg );
		void handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void wayPointsDealer();

		int wayPoint_counter;		
		geometry_msgs::Polygon _wayPoints;
		geometry_msgs::Point32 _goal;
		nav_msgs::Odometry _estimated_odom;
		nav_msgs::Odometry _odom;

};

#endif /* PLANNERXY_H */
