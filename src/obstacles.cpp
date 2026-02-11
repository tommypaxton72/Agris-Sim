#include "obstacles.h"



void Obstacles::AddObstacle(float x, float y, float radius) {
	Obstacle obs;
	obs.x = x;
	obs.y = y;
	obs.radius = radius;
	obstacles.push_back(obs);
}

void Obstacles::LoadConfig(const std::string& configPath) {
    YAML::Node config = YAML::LoadFile(configPath);

    // Loop through each obstacle in yaml and add to vector
    for (const auto& obs : config["obstacles"]) {
        Obstacle o;
        o.x      = obs["x"].as<float>();
        o.y      = obs["y"].as<float>();
        o.radius = obs["radius"].as<float>();
        obstacles.push_back(o);
    }
}
