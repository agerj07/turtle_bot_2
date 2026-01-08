#include "navigation/plannerxy.h"
#include "navigation/nodexy.h"
#include <cmath>
#include <vector>
#include <memory>
#include <algorithm>




PlannerXY::
PlannerXY() {}

PlannerXY::
~PlannerXY() {}

//look at the ocmapper oc map to determine obstacles	
//need to discretize the space 


void
PlannerXY::
handleSimulatedObstacles( const geometry_msgs::Polygon::ConstPtr& msg ){
	_obstacles = *msg;
	return;
}

void 
PlannerXY::
handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	_estimated_odom = *msg;
	return;
}

void 
PlannerXY::
handleOdom( const nav_msgs::Odometry::ConstPtr& msg){
	_odom = *msg;
	return;
}

void 
PlannerXY::
handleLookAhead(const geometry_msgs::PoseStamped::ConstPtr& msg){
	_lookAhead = *msg;
	return;
}

void 
PlannerXY::
handleGoal(const geometry_msgs::Point32::ConstPtr& msg){
	_goal = *msg;	
	return;
}

float
PlannerXY::
discrete(float a){
    float scaleFactor = 0.5;
    return std::round(a / scaleFactor) * scaleFactor;
}


bool 
PlannerXY::
is_at_goal() {
    std::shared_ptr<NodeXY> top = open_list.front();
    if ( (top->x) == (discrete(_goal.x)) && (top->y) == (discrete(_goal.y))) {
		closed_list.push_back(top);
        return true;
    } else {
        return false;
    }
}

bool 
PlannerXY::
Solve() {
    open_list.clear();
    closed_list.clear();
    path.clear();
	
	std::cout << "planner goalX: " << _goal.x << std::endl;
	std::cout << "planner goalY: " << _goal.y << std::endl;	
    std::shared_ptr<NodeXY> start_node = std::make_shared<NodeXY>(discrete(_estimated_odom.pose.pose.position.x), discrete(_estimated_odom.pose.pose.position.y));
    start_node->g = 0;
    start_node->h = sqrt( pow(_goal.x - _estimated_odom.pose.pose.position.x, 2) + pow(_goal.y - _estimated_odom.pose.pose.position.y, 2) );
    start_node->f = start_node->h; // f = g + h;
    start_node->parent = nullptr;
    open_list.push_back(start_node);

	
    float dx[8] = {0, 0.5, 0.5, 0.5, 0, -0.5, -0.5, -0.5};
    float dy[8] = {0.5, 0.5, 0, -0.5, -0.5, -0.5, 0, 0.5};
	//std::cout << "Start Node Finish" << std::endl;
	//std::cout << "X: " << start_node->x << std::endl;
	//std::cout << "Y: " << start_node->y << std::endl;

    while (!is_at_goal() ) {
		//std::cout << "Enters While Loop" << std::endl;
		//std::cout << "Planner X: " << start.pose.pose.position.x << std::endl;
		//std::cout << "Planner Y: " << start.pose.pose.position.y << std::endl;		
		if(!open_list.empty()){        
			std::shared_ptr<NodeXY> top_open = open_list.front(); // Take top of the open list
		    open_list.erase(open_list.begin());                   // Remove the first element from the open list
		    closed_list.push_back(top_open);                      // Push the top of the open list onto the closed list
		}
	
        std::shared_ptr<NodeXY>& current_node = closed_list.back(); // returns last element in closed list
		//std::cout << "Current Node Loaded" << std::endl;
			
        for (int i = 0; i < 8; i++) { // expand to each neighbor node
            float newX = dx[i] + current_node->x;
            float newY = dy[i] + current_node->y;
			bool obst = false;

			//obstacles check
			//std::cout << "Right before obstacle check" << std::endl;
			float LAx = _lookAhead.pose.position.x;
			float LAy = _lookAhead.pose.position.y;
			/*if(ocmap.checkMap(newX, newY, 0.35, 0) == false){
				std::cout << "Obstacle detected" << std::endl;				
				continue;
			}*/
			float deltaX = newX - current_node->x;
    		float deltaY = newY - current_node->y;
			float distance = sqrt( pow(deltaX, 2) + pow(deltaY, 2) );
			float stepSize = 0.05; // Adjust the step size as needed

			// Ensure we don't overshoot the neighbor node
			int numSamples = 10;
			float stepX = deltaX / numSamples;
			float stepY = deltaY / numSamples;

			for (int j = 0; j <= numSamples; j++) {
				float sampleX = current_node->x + j * stepX;
				float sampleY = current_node->y + j * stepY;

				// Check for obstacles at the sampled point
				//std::cout << "sampleX: " << sampleX << std::endl;
				//std::cout << "sampleY: " << sampleY << std::endl;
				if (ocmap.checkMap(sampleX, sampleY, 0.35, 0) == false) {
				    std::cout << "Obstacle detected at (" << sampleX << ", " << sampleY << ")" << std::endl;
					obst = true;
				    continue; // dont add node if an obstacle is detected
				}
			}
			
			// Sample points around newX and newY
			/*float sampleStep = 0.05; // Adjust the step size as needed
			float sampleRange = 0.4; // Adjust the sampling range as needed

			for (float sampleX = newX - sampleRange; sampleX <= newX + sampleRange; sampleX += sampleStep) {
				for (float sampleY = newY - sampleRange; sampleY <= newY + sampleRange; sampleY += sampleStep) {
					// Check for obstacles around sampleX and sampleY
					if (ocmap.checkMap(sampleX, sampleY, 0.4, 0) == false) {
						// Obstacle detected, skip this neighbor node
						std::cout << "Obstacle detected at (" << sampleX << ", " << sampleY << ")" << std::endl;
						continue;
					}
				}
			}*/
		  	if (obst == false){
		        std::shared_ptr<NodeXY> neighbor_node = std::make_shared<NodeXY>(newX, newY);
				//std::cout << "X: " << neighbor_node->x << std::endl;
				//std::cout << "Y: " << neighbor_node->y << std::endl;

		        if (std::abs(dx[i]) == 1 && std::abs(dy[i]) == 1) {
		            neighbor_node->g = sqrt(2.0) + current_node->g;
		        } else {
		            neighbor_node->g = 1 + current_node->g;
		        }

		        neighbor_node->h = sqrt( pow( _goal.x - newX, 2) + pow( _goal.y - newY, 2) );
		        neighbor_node->f = neighbor_node->h + neighbor_node->g;
		        neighbor_node->parent = current_node;

		        bool is_in_open = false;
		        for (std::shared_ptr<NodeXY> openNode : open_list) {
		            if (*openNode == *neighbor_node) {
		                is_in_open = true;
		                if ((neighbor_node->f) < (openNode->f)) {
		                    openNode->f = neighbor_node->f;
		                    openNode->g = neighbor_node->g;
		                    openNode->h = neighbor_node->h;
		                    openNode->parent = neighbor_node->parent;

							/*openNode.back()->f = neighbor_node.back()->f;
							openNode.back()->g = neighbor_node.back()->g;
							openNode.back()->h = neighbor_node.back()->h;
							openNode.back()->parent = neighbor_node.back()->parent;*/
							 
		                }
		                break;
		            }
		        }
		        if (!is_in_open) {
		            open_list.push_back(neighbor_node);
		        }
		    }
		}

        std::sort(open_list.begin(), open_list.end(), [](const std::shared_ptr<NodeXY>& a, const std::shared_ptr<NodeXY>& b) {
            return a->f < b->f;
        });
        // iterate through open list, organize the open list so the smallest f value is on top,
    }

	std::shared_ptr<NodeXY> path_node = closed_list.back();	//reconstruct the path
    while (path_node != nullptr) {
        path.push_front(path_node);
        path_node = path_node->parent;
    }

    // Print open_list
    /*for (std::shared_ptr<NodeXY>& node1 : open_list) {
        std::cout << "x: " << node1->x << std::endl;
        std::cout << "y: " << node1->y << std::endl;
        std::cout << "f: " << node1->f << std::endl;
        std::cout << "BP: " << node1->parent << std::endl;
    }*/

    // Print closed_list
	/*std::cout << "Closed List" << std::endl;
    for (std::shared_ptr<NodeXY>& node2 : closed_list) {
        std::cout << "x: " << node2->x << std::endl;
        std::cout << "y: " << node2->y << std::endl;
        std::cout << "f: " << node2->f << std::endl;
        std::cout << "BP: " << node2->parent << std::endl;
    }*/
	std::cout << "\nPath" << std::endl;
	for (std::shared_ptr<NodeXY>& node3 : path) {
        std::cout << "x: " << node3->x << std::endl;
        std::cout << "y: " << node3->y << std::endl;
        std::cout << "f: " << node3->f << std::endl;
        std::cout << "BP: " << node3->parent << std::endl;
    }
	

    return true;
}


