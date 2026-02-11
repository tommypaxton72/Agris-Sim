#ifndef WORLD_H
#define WORLD_H
#include "obstacles.h"
#include "robot.h"
#include <iostream>

struct WorldSize {
    float x;
    float y;
};

struct Physics {
    float friction;
};

class World {
public:
    World(const WorldSize& size);
    void Update(float dt);
    bool EdgeDetection(const pose& p);
    bool CollisionDetection(const pose& p);
	std::vector<Obstacle> CollisionClose(const pose& p, const Obstacles& obs, float threshold);
    // Getters for Renderer and Logger
    const pose& GetRobotPose() const;
    const robo& GetRobotConfig() const;
    const std::vector<Obstacle>& GetObstacles() const;
    const WorldSize& GetWorldSize() const;

    // Controller config loaded through World so robot stays private
    void LoadControllerConfig(const std::string& configPath);

    static constexpr float collisionThreshold = 50.f;

private:
    WorldSize worldSize;
    Obstacles obs;
    Robot robot;
    double threshold = 25;
};
#endif
