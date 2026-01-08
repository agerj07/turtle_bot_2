
#include "localization/ekf_localization.h"

#include "geometry_msgs/Point.h"
#include <cmath>
#include <iostream>

using namespace std;

geometry_msgs::Quaternion
yaw_to_quaternion( const double& yaw ){

	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos( yaw / 2.0 );
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin( yaw / 2.0 );

	return quaternion;
}

EKF_Localization::
EKF_Localization( const Eigen::VectorXd& alpha, const Eigen::MatrixXd& q ) : _u(), _landmarks(), _z(), _mu( Eigen::VectorXd::Zero( 3 ) ), _sigma( Eigen::MatrixXd::Zero( 3, 	3 ) ), _alpha( alpha ), _q( q ) {
	
}

EKF_Localization::
~EKF_Localization() {
}

void
EKF_Localization::
handle_command( const geometry_msgs::Twist::ConstPtr& msg ){
	_u = *msg;
	return;
}

void
EKF_Localization::
handle_odometry( const nav_msgs::Odometry::ConstPtr& msg ){
	_u = msg->twist.twist;
	return;
}

void
EKF_Localization::
handle_landmarks( const perception::Landmarks::ConstPtr& msg ){
	for( unsigned int i = 0; i < msg->landmarks.size(); i++ ){
		map< int, geometry_msgs::Point >::iterator it_landmark = _landmarks.find( msg->landmarks[ i ].signature );

		if( it_landmark != _landmarks.end() ){
			it_landmark->second.x = msg->landmarks[ i ].x;
			it_landmark->second.y = msg->landmarks[ i ].y;
		} else {
			geometry_msgs::Point landmark_posistion;
    		landmark_posistion.x = msg->landmarks[i].x;
    		landmark_posistion.y = msg->landmarks[i].y;
    		landmark_posistion.z = 0.0; 

			_landmarks.insert( pair< int, geometry_msgs::Point >( msg->landmarks[ i ].signature, landmark_posistion ) );
		}
	}
	return;
}

void
EKF_Localization::
handle_observations( const perception::Observations::ConstPtr& msg ){
	_z = *msg;
	return;
}

void
EKF_Localization::
step( const double& dt ){
	// implement motion model step for time dt (make sure to account for zero angular velocity!)
	/**
	* x(0) = x position
	* x(1) = y position
	* x(2) = heading
	* u2(0) = linear velocity (v)
	* u2(1) = angular_velocity (w)
	*/	
	//Eigen::VectorXd _x = Eigen::VectorXd::Zero( 3 );
	Eigen::VectorXd _u2 = Eigen::VectorXd::Zero( 2 );
	_u2(0) = _u.linear.x;	//check to make sure sub to
	_u2(1) = _u.angular.z;
	
	//std::cout << "Linear Velocity: " << _u2(0) << std::endl;
	//std::cout << "Angular Velocity: " << _u2(1) << std::endl;
	/*_x(0) = _x(0) + (_u2(0) * cos(_x(2)) * dt);
	_x(1) = _x(1)+ (_u2(0) * sin(_x(2)) * dt);
	_x(2) = _x(2) + _u2(1) * dt;*/
	
	//want to keep bearing between -pi and pi for sim
	/*while(_x(2) > M_PI){
		_x(2) = _x(2) -  2 * M_PI;
	
	}
	while(_x(2) < (-1 * M_PI)){
		_x(2) = _x(2) + 2 * M_PI;
	}*/
		

	Eigen::MatrixXd _Gt = Eigen::MatrixXd::Zero(3, 3);
	Eigen::MatrixXd _Vt = Eigen::MatrixXd::Zero(3, 2);
	Eigen::MatrixXd _Mt = Eigen::MatrixXd::Zero(2, 2);
	Eigen::VectorXd _muBar = Eigen::VectorXd::Zero( 3 );
	Eigen::MatrixXd _sigmaBar = Eigen::MatrixXd::Zero(3, 3);
	Eigen::VectorXd _muBarAdd = Eigen::VectorXd::Zero( 3 );	

	if((_u2(1) < 0.00001) && (_u2(1) > -0.00001)) {	//zero angular velocity (or close to zero)
		_Gt(0, 0) = 1;
		_Gt(0, 1) = 0;
		_Gt(0, 2) = (-1 * _u2(0) * sin(_mu(2)) * dt );
		_Gt(1, 0) = 0;
		_Gt(1, 1) = 1;
		_Gt(1, 1) = _u2(0) * cos( _mu(2)) * dt;
		_Gt(2, 0) = 0;
		_Gt(2, 1) = 0;
		_Gt(2, 2) = 1;


		_Vt(0, 0) = dt * cos(_mu(2));
		_Vt(0, 1) = 0;
		_Vt(1, 0) = dt * sin(_mu(2));
		_Vt(1, 1) = 0;
		_Vt(2, 0) = 0;
		_Vt(2, 1) = 0;		//could be zero from math

		_Mt(0, 0) = (_alpha(0) * _u2(0) * _u2(0)) + (_alpha(1) * _u2(1) * _u2(1));
		_Mt(0, 1) = 0;
		_Mt(1, 0) = 0; 
		_Mt(1, 1) = (_alpha(2) * _u2(0) * _u2(0)) + (_alpha(3) * _u2(1) * _u2(1));

		_muBar(0) = _mu(0) + _u2(0) * dt * cos(_mu(2));
		_muBar(1) = _mu(1) + _u2(0) * dt * sin(_mu(2));
		_muBar(2) = _mu(2);
		//std::cout << "muBar: " << _muBar << std::endl;
		//3X1 = 3X1 + 3X1
		_sigmaBar = ( _Gt * _sigma * _Gt.transpose() ) + ( _Vt * _Mt * _Vt.transpose() );
		//3X3 = 3X3 * 3X3 * 3X3 + 3X2 * 2X2 * 2X3
	}
	else{
		_Gt(0, 0) = 1;
		_Gt(0, 1) = 0;
		_Gt(0, 2) = (-1 * (_u2(0) / _u2(1)) * cos(_mu(2))) + ((_u2(0) / _u2(1)) * cos(_mu(2) + _u2(1) * dt));
		_Gt(1, 0) = 0;
		_Gt(1, 1) = 1;
		_Gt(1, 1) = (-1 * (_u2(0) / _u2(1)) * sin(_mu(2))) + ((_u2(0) / _u2(1)) * sin(_mu(2) + _u2(1) * dt));
		_Gt(2, 0) = 0;
		_Gt(2, 1) = 0;
		_Gt(2, 2) = 1;


		_Vt(0, 0) = (-1 * sin(_mu(2)) + sin(_mu(2) + _u2(1) * dt)) / _u2(1);
		_Vt(0, 1) = ((_u2(0) * (sin(_mu(2)) - sin(_mu(2) + _u2(1) * dt)) / (_u2(1) * _u2(1))) + (_u2(0) * cos(_mu(2) + _u2(1) * dt) * dt) / _u2(1));
		_Vt(1, 0) = (cos(_mu(2)) - cos(_mu(2) + _u2(1) * dt)) / _u2(1);
		_Vt(1, 1) = ((-1 * _u2(0) * (cos(_mu(2)) - cos(_mu(2) + _u2(1) * dt)) / (_u2(1) * _u2(1))) + (_u2(0) * sin(_mu(2) + _u2(1) * dt) * dt) / _u2(1));
		_Vt(2, 0) = 0;
		_Vt(2, 1) = dt;

		_Mt(0, 0) = (_alpha(0) * _u2(0) * _u2(0)) + (_alpha(1) * _u2(1) * _u2(1));
		_Mt(0, 1) = 0;
		_Mt(1, 0) = 0; 
		_Mt(1, 1) = (_alpha(2) * _u2(0) * _u2(0)) + (_alpha(3) * _u2(1) * _u2(1));


		_muBarAdd(0) = (-1 * (_u2(0) / _u2(1)) * sin(_mu(2))) + ((_u2(0) / _u2(1)) * sin(_mu(2) + _u2(1) * dt));
		_muBarAdd(1) = ((_u2(0) / _u2(1)) * cos(_mu(2))) - ((_u2(0) / _u2(1)) * cos(_mu(2) + _u2(1) * dt));
		_muBarAdd(2) = _u2(1) * dt;
		//std::cout << "_muBarAdd: " << _muBarAdd << std::endl; 
		_muBar = _mu + _muBarAdd;
		while(_muBar(2) > M_PI){
				_muBar(2) = _muBar(2) -  2 * M_PI;
		}
		while(_muBar(2) <  (-1 * M_PI) ){
				_muBar(2) = _muBar(2) + 2 * M_PI;
		}
		
		//std::cout << "muBar: " << _muBar << std::endl;
		//3X1 = 3X1 + 3X1

		_sigmaBar = ( _Gt * _sigma * _Gt.transpose() ) + ( _Vt * _Mt * _Vt.transpose() );
		//3X3 = 3X3 * 3X3 * 3X3 + 3X2 * 2X2 * 2X3

	}

	Eigen::MatrixXd _I = Eigen::MatrixXd::Identity(3, 3);
	Eigen::VectorXd _zHat = Eigen::VectorXd::Zero( 3 );
	Eigen::MatrixXd _Ht = Eigen::MatrixXd::Zero(3, 3);
	Eigen::MatrixXd _St = Eigen::MatrixXd::Zero(3, 3);
	Eigen::MatrixXd _Kt = Eigen::MatrixXd::Zero(3, 3);
	Eigen::VectorXd _zVec = Eigen::VectorXd::Zero( 3 );
	Eigen::VectorXd Innovation = Eigen::VectorXd::Zero( 3 );


	for( unsigned int i = 0; i < _z.observations.size(); i++ ){
	// implement measurement model step for all observations
	map< int, geometry_msgs::Point >::iterator it_landmark = _landmarks.find( _z.observations[ i ].signature );
		if( it_landmark != _landmarks.end() ){	//can do ekf loop within this for loop
			int landmark_signature = it_landmark -> first;			
			double landmark_mx = it_landmark -> second.x;
			double landmark_my = it_landmark -> second.y;
			

			double q = ((landmark_mx - _muBar(0)) * (landmark_mx - _muBar(0))) + ((landmark_my - _muBar(1)) * (landmark_my - _muBar(1)));
			_zHat(0) = sqrt(q);
			_zHat(1) = atan2(landmark_my - _muBar(1), landmark_mx - _muBar(0)) - _muBar(2);
			_zHat(2) = landmark_signature;
		

			_Ht(0, 0) = -1 * ((landmark_mx - _muBar(0)) / (sqrt(q)));
			_Ht(0, 1) = -1 * ((landmark_my - _muBar(1)) / (sqrt(q)));
			_Ht(0, 2) = 0;
			_Ht(1, 0) = (landmark_my - _muBar(1)) / q;
			_Ht(1, 1) = -1 * ((landmark_mx - _muBar(0)) / q);
			_Ht(1, 2) = -1;
			//The third row is already filled in to be zeroes

			_St = (_Ht * _sigmaBar * _Ht.transpose()) + _q;
			//3X3 = (3X3 * 3X3 * 3X3) + 3X3
	
			_Kt = _sigmaBar * _Ht.transpose() * _St.inverse();
			//3X3 = 3X3 * 3X3 * 3X3
			//put the observation data in a vector
			
			_zVec(0) = _z.observations[i].range;
			_zVec(1) = _z.observations[i].bearing;
			_zVec(2) = _z.observations[i].signature;
		
			Innovation(0) = _zVec(0) - _zHat(0);
			Innovation(1) = _zVec(1) - _zHat(1);
			Innovation(2) = _zVec(2) - _zHat(2);	

			while(Innovation(1) > M_PI){
				Innovation(1) = Innovation(1) -  2 * M_PI;
			}
			while(Innovation(1) <  (-1 * M_PI) ){
				Innovation(1) = Innovation(1) + 2 * M_PI;
			}
			
			_muBar = _muBar + ( (_Kt) * (Innovation) );	//bound angle error to -pi and pi

			while(_muBar(2) > M_PI){
				_muBar(2) = _muBar(2) -  2 * M_PI;
			}
			while(_muBar(2) <  (-1 * M_PI) ){
				_muBar(2) = _muBar(2) + 2 * M_PI;
			}
			/*while(_muBar(2) > M_PI){
				_muBar(2) = _muBar(2) -  2 * M_PI;
	
			}
			while(_muBar(2) < (-1 * M_PI)){
				_muBar(2) = _muBar(2) + 2 * M_PI;
			}*/
			//3X1 = 3X3 * (3X1 - 3X1)
			_sigmaBar = (_I - (_Kt * _Ht)) * _sigmaBar;		//make _z a seperate vector and then do calc.
			//3X3 = (3X3 - (3X3 * 3X3)) * 3X3

		}
	}	
	_mu = _muBar;
	_sigma = _sigmaBar;


	// clear past observations
	_z.observations.clear();
	return;
}

nav_msgs::Odometry
EKF_Localization::
estimated_odometry( void )const{

	nav_msgs::Odometry msg;
	msg.pose.pose.position.x = _mu( 0 );
	msg.pose.pose.position.y = _mu( 1 );
	msg.pose.pose.orientation = yaw_to_quaternion( _mu( 2 ) );
	/*std::cout << "x: " << _mu(0) << std::endl;
	std::cout << "y: " << _mu(1) << std::endl;
	std::cout << "Angle: " << yaw_to_quaternion(_mu(2)) << std::endl;*/
	return msg;
}

