#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <vector>

struct Obstacle {
	float x;
	float y;
	float radius;
	
};

class Obstacles {
public:
	std::vector<Obstacle> obstacles;
	void AddObstacle(float x, float y, float radius);
private:



}



#endif
