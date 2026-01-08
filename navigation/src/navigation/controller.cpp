#include "navigation/controller.h"

double
quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
	return atan2( 2.0 * ( quaternion.w * quaternion.z + quaternion.x * quaternion.y ), 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ) );
}



Controller::
Controller(){
	lookAhead_index = 0;	
}

Controller::
~Controller(){}

void 
Controller::
handlePath( const nav_msgs::Path::ConstPtr& msg ){
	_path = *msg;
	controlCalc();
}

void 
Controller::
handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg){
	_estimated_odom = *msg;
}

void 
Controller::
handleOdom( const nav_msgs::Odometry::ConstPtr& msg){
	_odom = *msg;
}

/*void
Controller::
controlCalc(){
	//std::cout << "Enter calc function" << std::endl;
	float posX = _estimated_odom.pose.pose.position.x;
	float posY = _estimated_odom.pose.pose.position.y;
	float theta = quaternion_to_yaw( _estimated_odom.pose.pose.orientation);
	std::cout << "Robot angle: " << theta << std::endl;
	std::cout << "PosX: " << posX << std::endl;
	std::cout << "PosY: " << posY << std::endl;	
	
	if(_path.poses.empty()){
		std::cout << "No Path" << std::endl;
		move_control.linear.x = 0;
		move_control.angular.z = 0;	
		return;
	}
	
	lookAhead_pub = _path.poses[lookAhead_index];
	float lookAheadX = _path.poses[lookAhead_index].pose.position.x;
	float lookAheadY = _path.poses[lookAhead_index].pose.position.y;
	std::cout << "lookAheadX: " << lookAheadX << std::endl;
	std::cout << "lookAheadY: " << lookAheadY << std::endl;	

	float angle = atan2(lookAheadY - posY, lookAheadX - posX);
	float angle_diff = angle - theta;
	//std::cout << "Control Calc Called" << std::endl;
	std::cout << "Angle Diff: " << angle_diff << std::endl;
	if (angle_diff > 0.1){
		std::cout<< "Angle movement Pos" << std::endl;
		move_control.angular.z = 0.5;
		move_control.linear.x = 0;
	}
	else if(angle_diff < -0.1){
		std::cout<< "Angle movement Neg" << std::endl;
		move_control.angular.z = -0.5;
		move_control.linear.x = 0;
	}


	else if( (sqrt( pow(lookAheadX - posX, 2) + pow(lookAheadY - posY, 2))) > 0.1){
		std::cout<< "Linear movement" << std::endl;
		move_control.linear.x = 0.5;
		move_control.angular.z = 0;	

	}
	
	
	if( sqrt( pow(lookAheadX - posX, 2) + pow(lookAheadY - posY, 2)) < 0.1 ){
		lookAhead_index++;
		
	}
	

	
}*/

void
Controller::
controlCalc(){
	//std::cout << "Enter calc function" << std::endl;
	float posX = _estimated_odom.pose.pose.position.x;
	float posY = _estimated_odom.pose.pose.position.y;
	float theta = quaternion_to_yaw( _estimated_odom.pose.pose.orientation);
	
	if(_path.poses.empty()){
		std::cout << "No Path" << std::endl;
		move_control.linear.x = 0;
		move_control.angular.z = 0;
		lookAhead_index = 0;	
		return;
	}
	std::cout << "LookAhead Index: " << lookAhead_index << std::endl;
	lookAhead_pub = _path.poses[lookAhead_index];
	float lookAheadX = _path.poses[lookAhead_index].pose.position.x;
	float lookAheadY = _path.poses[lookAhead_index].pose.position.y;
	std::cout << "lookAheadX: " << lookAheadX << std::endl;
	std::cout << "lookAheadY: " << lookAheadY << std::endl;	

	float angle = atan2(lookAheadY - posY, lookAheadX - posX);
	float angle_diff = angle - theta; //make angle_dif between -pi and pi
	if( angle_diff < -M_PI ){
			angle_diff = angle_diff + 2.0 * M_PI;
	}else if ( angle_diff > M_PI ){
			angle_diff = angle_diff - 2.0 * M_PI;
	}
	//std::cout << "Control Calc Called" << std::endl;
	std::cout << "Angle Diff: " << angle_diff << std::endl;
	if (angle_diff > 0.2){	//8.59 degrees
		move_control.angular.z = 0.5;
		move_control.linear.x = 0;
	}
	else if(angle_diff < -0.2){	//-8.59 degrees
		move_control.angular.z = -0.5;
		move_control.linear.x = 0;
	}


	else if( (sqrt( pow(lookAheadX - posX, 2) + pow(lookAheadY - posY, 2))) > 0.1){
		move_control.linear.x = 0.5;
		move_control.angular.z = 0;	

	}
	
	
	if( sqrt( pow(lookAheadX - posX, 2) + pow(lookAheadY - posY, 2)) < 0.1 ){
		lookAhead_index++;
		
	}
	float path_array_size = _path.poses.size() - 1;
	float finalPathX = _path.poses[path_array_size].pose.position.x;
	float finalPathY = _path.poses[path_array_size].pose.position.y;
	if(!_path.poses.empty() && sqrt( pow(finalPathX - posX, 2) + pow(finalPathY - posY, 2)) < 0.1){
		lookAhead_index = 0;
	}
	

	
}

