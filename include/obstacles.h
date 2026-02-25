#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "types.h"


class Obstacles {
public:
  // Getter Function
	const std::vector<Obstacle>& GetObstacles() const { return obstacles; }
    void AddObstacle(float x, float y, float radius);
    void LoadConfig(const std::string& configPath);
private:
	std::vector<Obstacle> obstacles;


};



#endif
