#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

struct Obstacle {
	float x;
	float y;
	float radius;
	
};

class Obstacles {
public:
	std::vector<Obstacle> obstacles;
    void AddObstacle(float x, float y, float radius);
    void LoadConfig(const std::string& configPath);
private:



};



#endif
