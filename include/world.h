#ifndef WORLD_H
#define WORLD_H

#include "obstacles.h"
#include "robot.h"
#include <iostream>
#include "types.h"


class World {
    public:
	World(const WorldSize& size);

	// Main update function for updating the world.
    void Update(float dt);

	// Collision detection functions
    bool EdgeDetection(const pose& p);
    bool CollisionDetection(const pose& p);

    
    // Getters for Renderer and Logger
    const pose& GetRobotPose() const;
    const robo& GetRobotConfig() const;
    const std::vector<Obstacle>& GetObstacles() const;
    const WorldSize& GetWorldSize() const;
    
  private:
	// Worldsize{x, y}
    WorldSize worldSize;
	// Obstacles class
    Obstacles obs;
	// Robot class 
    Robot robot;

    // Gets all close obstacles to pass to collision detection
    std::vector<Obstacle> CollisionClose(const pose& p, const Obstacles& obs);
    
	point Transform(float vecX, float vecY, float theta);
    float Clamp(float input, float min, float max);
	
};
#endif
