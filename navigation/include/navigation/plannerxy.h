#ifndef PLANNERXY_H
#define PLANNERXY_H

#include <iostream>
#include <deque>
#include <vector>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point32.h"
#include "navigation/nodexy.h"
#include "geometry_msgs/Polygon.h"
#include "mapper/ocmap.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"


class PlannerXY{
	public:
		PlannerXY();
		virtual ~PlannerXY();			
		bool Solve();
		void discretization();
		bool is_at_goal();
		void handleSimulatedObstacles( const geometry_msgs::Polygon::ConstPtr& msg );
		void handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg);
		void handleLookAhead(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void handleGoal(const geometry_msgs::Point32::ConstPtr& msg);
		float discrete(float a);

		std::vector<std::shared_ptr<NodeXY>> open_list;
		std::vector<std::shared_ptr<NodeXY>> closed_list;		
		std::deque<std::shared_ptr<NodeXY>> path;
		geometry_msgs::Polygon _obstacles;
		nav_msgs::Odometry _estimated_odom;
		nav_msgs::Odometry _odom;
		geometry_msgs::PoseStamped _lookAhead;
		geometry_msgs::Point32 _goal;
		OCMap ocmap;


};

#endif /* PLANNERXY_H */
