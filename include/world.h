#ifndef WORLD_H
#define WORLD_H
#include "obstacles.h"
#include "robot.h"
#include <iostream>

struct WorldSize {
    float x;
    float y;
};

class World {
    public:
	World(const WorldSize& size);

	// Main update function for updating the world.
    void Update(float dt);

	// Collision detection functions
    bool EdgeDetection(const pose& p);
    bool CollisionDetection(const pose& p);
    
    // Vector for storing all obstacles loaded from YAML file.
    std::vector<Obstacle> CollisionClose(const pose& p, const Obstacles& obs, float threshold);

    // Getters for Renderer and Logger
    const pose& GetRobotPose() const;
    const robo& GetRobotConfig() const;
    const std::vector<Obstacle>& GetObstacles() const;
    const WorldSize& GetWorldSize() const;

    // Controller config loaded through World so robot stays private
	// I might change this later
    void LoadControllerConfig(const std::string& configPath);

  private:
	// Worldsize{x, y}
    WorldSize worldSize;
	// Obstacles{x, y, radius}
    Obstacles obs;
	// Robot{width, length, maxVelocity, wheelDistance}
    Robot robot;
    
};
#endif
