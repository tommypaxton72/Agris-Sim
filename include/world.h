#ifndef WORLD_H
#define WORLD_H


#include <iostream>
#include <vector>
#include "robot.h"

struct Obstacle {
	float x;
	float y;
	float radius;
	
};

struct Physics
{
	float friction;
};

	
class World {
public:
	World(float max_x, float max_y);
	void init();
	void AddObstacle(float x, float y, float radius);
	bool CheckCollision();
	std::vector<Obstacle> obstacles;
	float worldX;
	float worldY;
	

private:
};

#endif
