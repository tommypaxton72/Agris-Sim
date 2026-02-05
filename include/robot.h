#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>


struct pose 
{
	float x;
	float y;
	float theta;
}
	




class Robot 
{
public:
	Robot();
	pose p;
	pose UpdatePose();
	void KinematicUpdate();
	


private:



}

