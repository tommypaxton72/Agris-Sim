#include "world.h"






World::World(float max_x, float max_y) : worldX(max_x), worldY(max_y) {}






void AddObstacle(float x, float y, float radius) {
	Obstacle obs;
	obs.x = x;
	obs.y = y;
	obs.radius = radius;
	obstacles.pushback(obs);
}

