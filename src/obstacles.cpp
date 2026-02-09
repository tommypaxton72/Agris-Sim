#include "obstacles.h"



void AddObstacle(float x, float y, float radius) {
	Obstacle obs;
	obs.x = x;
	obs.y = y;
	obs.radius = radius;
	obstacles.pushback(obs);
}
