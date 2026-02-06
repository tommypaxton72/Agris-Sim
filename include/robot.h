#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <eigen> // Matrix math?

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
	void Update(float dt);
	pose p;
	pose UpdatePose();
	void KinematicUpdate();



private:



};


#endif
