#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "types.h"


class Obstacles {
public:
	// Do we want this to be public?
	// Vector of Obstacle
    std::vector<Obstacle> obstacles;

    void AddObstacle(float x, float y, float radius);
    void LoadConfig(const std::string& configPath);
private:



};



#endif
