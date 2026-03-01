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
// Getters
	const pose& GetRobotPose() const { return robot.GetPose(); };
	const robo& GetRobotConfig() const { return robot.GetRobo(); };
	const std::vector<Obstacle>& GetObstacles() const { return obs.GetObstacles(); };
	const WorldSize& GetWorldSize() const { return worldSize; };
    const LidarData& GetLidarData() const { return robot.GetLidarData(); };
	const RANSACLine& GetRightLine() const { return robot.GetRightLine(); };
    const RANSACLine& GetLeftLine() const { return robot.GetLeftLine(); };
	const MotorControl& GetLeftMotor() const { return robot.GetLeftMotor(); };
    const MotorControl& GetRightMotor() const { return robot.GetRightMotor(); };
    const float& GetLeftVel() const { return robot.GetLeftVel(); };
    const float& GetRightVel() const {
        return robot.GetRightVel();
    };
	const DataLayer& GetDataLayer() const { return robot.GetDataLayer(); };
	const Debug& GetDebug() const {return robot.GetDebug(); };
    void PrintLidarDebug();
  private:
	// Worldsize{x, y}
    WorldSize worldSize;
	// Obstacles class
    Obstacles obs;
	// Robot class 
    Robot robot;

    // Gets all close obstacles to pass to collision detection
    std::vector<Obstacle> CollisionClose(const pose& p);
    
	point Transform(float vecX, float vecY, float theta);
    float Clamp(float input, float min, float max);
	
};
#endif
