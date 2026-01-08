/*
#include <iostream>
#include "ros/ros.h"

#include "perception/Landmarks.h"

using namespace std;

int
main( int argc, char* argv[] ){

	perception::Landmarks landmarks;

	unsigned int num_landmarks = 50;
	double min_x = -10.0;
	double max_x = 10.0;
	double min_y = -10.0;
	double max_y = 10.0;
	//double min_signature = 0.3;
	//double max_signature = 0.7;

	for( unsigned int i = 0; i < num_landmarks; i++ ){
		landmarks.landmarks.push_back( perception::Landmark() );
		landmarks.landmarks.back().x = min_x + ( double )( rand() % 101 ) / ( 100.0 ) * ( max_x - min_x );
		landmarks.landmarks.back().y = min_y + ( double )( rand() % 101 ) / ( 100.0 ) * ( max_y - min_y );
		landmarks.landmarks.back().signature = i;
	}
	ros::init( argc, argv, "landmarks_publisher_node" );
	ros::NodeHandle node_handle;
	ros::Publisher landmarks_publisher = node_handle.advertise< perception::Landmarks landmarks >( "landmarks", 1, true );

	landmarks_publisher.publish( landmarks );

	sleep( 1 );

	return 0;
	}*/

#include <iostream>
#include "ros/ros.h"

#include "perception/Landmarks.h"

using namespace std;

int
main( int argc, char* argv[] ){

  perception::Landmarks landmarks;

  ros::init( argc, argv, "landmarks_publisher_node" );
  ros::NodeHandle node_handle;
  ros::Publisher landmarks_publisher = node_handle.advertise< perception::Landmarks >( "landmarks", 1, true );

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = 1.85383;
  landmarks.landmarks.back().y = -1.08811;
  landmarks.landmarks.back().signature = 4;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = -5.38034;
  landmarks.landmarks.back().y = -0.800223;
  landmarks.landmarks.back().signature = 25;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = 4.64244;
  landmarks.landmarks.back().y = 8.07264;
  landmarks.landmarks.back().signature = 26;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = -0.656394;
  landmarks.landmarks.back().y = -1.59951;
  landmarks.landmarks.back().signature = 27;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = 4.86037;
  landmarks.landmarks.back().y = 0.856292;
  landmarks.landmarks.back().signature = 29;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = 5.24392;
  landmarks.landmarks.back().y = 7.36414;
  landmarks.landmarks.back().signature = 30;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = 5.2232;
  landmarks.landmarks.back().y = 3.45311;
  landmarks.landmarks.back().signature = 31;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = -6.12557;
  landmarks.landmarks.back().y = 8.17437;
  landmarks.landmarks.back().signature = 33;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = 5.25312;
  landmarks.landmarks.back().y = 4.99968;
  landmarks.landmarks.back().signature = 34;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = 0.795362;
  landmarks.landmarks.back().y = 8.27818;
  landmarks.landmarks.back().signature = 35;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = -0.305513;
  landmarks.landmarks.back().y = -1.61749;
  landmarks.landmarks.back().signature = 36;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = -2.62353;
  landmarks.landmarks.back().y = -0.831462;
  landmarks.landmarks.back().signature = 37;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = -6.75229;
  landmarks.landmarks.back().y = 1.15514;
  landmarks.landmarks.back().signature = 39;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = -5.60162;
  landmarks.landmarks.back().y = 9.12503;
  landmarks.landmarks.back().signature = 41;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = -1.36084;
  landmarks.landmarks.back().y = 8.20554;
  landmarks.landmarks.back().signature = 42;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = 5.19547;
  landmarks.landmarks.back().y = 4.38958;
  landmarks.landmarks.back().signature = 43;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = -6.46812;
  landmarks.landmarks.back().y = 3.58953;
  landmarks.landmarks.back().signature = 45;

  landmarks.landmarks.push_back( perception::Landmark() );
  landmarks.landmarks.back().x = 1.84313;
  landmarks.landmarks.back().y = 8.09644;
  landmarks.landmarks.back().signature = 48;

  sleep( 1 );

  landmarks_publisher.publish( landmarks );
  
  sleep( 1 );

  return 0;
}

