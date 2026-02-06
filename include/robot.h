#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <eigen> // Matrix math?
#include "world.h" // Dont know if i want to make a seperate lib for obstacles?

struct robo {
	float width;
	float length;
	float height;
};


struct pose 
{
	float x;
	float y;
	float theta;
};






class Robot 
{
public:
	Robot(float startX, float startY, float startTheta);
	robo r;
	pose p;
	
	void Update(float dt);
	pose UpdatePose();
	void KinematicUpdate();


private:



};


#endif
