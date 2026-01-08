#include "navigation/nodexy.h"

#include <iostream>

NodeXY::
NodeXY(float _x, float _y){
	x = _x;
	y = _y;
}

NodeXY::
~NodeXY(){}

NodeXY::
NodeXY(const NodeXY& other){
	x = other.x;
	y = other.y;
	g = other.g;
	h = other.h;
	f = other.f;
	parent = other.parent;


}

NodeXY&
NodeXY::
operator=(const NodeXY& other){
	if(this != &other){
		x = other.x;
		y = other.y;
		g = other.g;
		h = other.h;
		f = other.f;
		parent = other.parent;
	}
	return *this;
}

bool 
NodeXY::
operator ==(const NodeXY& other) const {
	if (x == other.x && y == other.y){
		return true;
	}
	else{
		return false;
	}
	
}

