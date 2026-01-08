#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "navigation/plannerxy.h"
#include "navigation/nodexy.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include <cmath>
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

class Controller{
	public:
		Controller();
		virtual ~Controller();
		void handlePath( const nav_msgs::Path::ConstPtr& msg );
		void handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg);
		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg);
		void controlCalc();


		//PlannerXY planner;
		geometry_msgs::Twist move_control;		
		nav_msgs::Odometry _estimated_odom;
		nav_msgs::Odometry _odom;			
		nav_msgs::Path _path;
		int lookAhead_index;
		PlannerXY planner;	
		geometry_msgs::PoseStamped lookAhead_pub;	
};
