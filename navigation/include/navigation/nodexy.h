#ifndef NODEXY_H
#define NODEXY_H

#include <iostream>
#include <memory>


#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point32.h"

class NodeXY{
	
	public:
		NodeXY(float _x, float _y);		
		NodeXY(const NodeXY& other);	
		virtual ~NodeXY();
		NodeXY& operator=(const NodeXY& other);
		bool operator ==(const NodeXY& other) const;

		float x;
		float y;

		float g;
		float h;
		float f;
		std::shared_ptr<NodeXY> parent;//back pointer
};

#endif /* NODEXY_H */
	
