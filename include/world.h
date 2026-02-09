#ifndef WORLD_H
#define WORLD_H


#include <iostream>
#include "robot.h"
#include "obstacles.h"


struct Physics
{
	float friction;
};

	
class World {
public:
	World(float max_x, float max_y);
	float worldX;
	float worldY;

	void init();
	bool CheckCollision();
	float worldX;
	float worldY;
	

private:
};

#endif
