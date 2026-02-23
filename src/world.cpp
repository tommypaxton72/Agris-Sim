#include "world.h"
#include <yaml-cpp/yaml.h>






// Most of the YAML stuff is from claude. Thanks Claude.
World::World(const WorldSize& size) : worldSize(size) {

    try {
        robot.LoadConfig();
	} catch  (const YAML::BadFile& e) {
        std::cerr << "Could not load robot.yaml: " << e.what() << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing robot.yaml: " << e.what() << std::endl;
    }

    try {
        obs.LoadConfig("config/obstacles.yaml");
    } catch (const YAML::BadFile& e) {
        std::cerr << "Could not load obstacles.yaml: " << e.what() << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing obstacles.yaml: " << e.what() << std::endl;
    }
}

// Might move this elsewhere
void World::LoadControllerConfig(const std::string& configPath) {
    robot.controller.LoadConfig(configPath);
}

// Updates the world
// Takes in Controller inputs
// Creates new robot pose
// Before robot pose is set it checks for collision
// Sets Pose
void World::Update(float dt) {
    // Can switch this out with different inputs.
    robot.controller.Update();

    float leftStick  = robot.controller.GetLeftStick();
    float rightStick = robot.controller.GetRightStick();
    // Just make sure to change robot.UpdatePose() To accept correct inputs. Its an overloaded function.
    robot.lidar.GetScan(p, obs);
	robot.RunControl(data);
    pose testPose = robot.UpdatePose(dt, leftStick, rightStick);
	// Before updating check if collision is false
    if (!CollisionDetection(testPose) && !EdgeDetection(testPose)) {
		// Set pose
        robot.SetPose(testPose);
    }
}

// Returns true if collision with edge is detected and false if free. 
bool World::EdgeDetection(const pose& p) {

    if (p.x + robot.r.width / 2 <= worldSize.x &&
        p.x + robot.r.width / 2 >= 0 &&
        p.y + robot.r.width / 2 <= worldSize.y &&
        p.y + robot.r.width / 2 >= 0) {
        return false;
    }
    return true;
}

// Narrow phase collision detection using local pose transformation AABB vs Circle.
// Returns true if there is a collision and false if there isnt one.
bool World::CollisionDetection(const pose& p) {
    
    std::vector<Obstacle> nearby = CollisionClose(p, obs);
    
    for (const auto& obstacle : nearby) {
		// Get vector from robot to obstacle.
		float gDistX = p.x - obstacle.x;
		float gDistY = p.y - obstacle.y;
		// Transform from global frame to robots local frame.
		point local = Transform(gDistX, gDistY, -p.theta);
		// Find closest point on robot edge to obstacle center.
		float closestX = Clamp(local.x, -1 * robot.r.width / 2, robot.r.width / 2);
		float closestY = Clamp(local.y, -1 * robot.r.length / 2, robot.r.length / 2);
		// Get distance from nearest point on robot to center of obstacle.
		float diffX = closestX - local.x;
		float diffY = closestY - local.y;
		float distSquared = diffX * diffX + diffY * diffY;
		// check if this distance is less than the radius of the circle
		if (distSquared < obstacle.radius * obstacle.radius) return true;
    }
    return false;
}

// Broad phase check - returns only obstacles within robot length + obstacle radius * 2
// But I might change this at some point?
std::vector<Obstacle> World::CollisionClose(const pose& p, const Obstacles& obs) {
    std::vector<Obstacle> nearby;

    for (int i = 0; i < obs.obstacles.size(); i++) {
        double dx = p.x - obs.obstacles[i].x;
        double dy = p.y - obs.obstacles[i].y;
        double distSquared = dx * dx + dy * dy;
		// robot.length is used because I imagine length will usually be larger than width.
		double threshold = robot.r.length + obs.obstacles[i].radius; 
        if (distSquared <= threshold * threshold) {
            nearby.push_back(obs.obstacles[i]);
        }
    }
    return nearby;
}

// Rotation transformation
 point World::Transform(float vecX, float vecY, float theta) {
	 point newP;
	 float c = std::cos(theta);
	 float s = std::sin(theta);
	 newP.x = (vecX * c) - (vecY * s);
	 newP.y = (vecX * s) + (vecY * c);
	 return newP;
}

// Clamp
float World::Clamp(float input, float min, float max) {
	if (input > max) return max;
	if (input < min) return min;
	return input;
}

// Getters
const pose& World::GetRobotPose() const { return robot.p; }
const robo& World::GetRobotConfig() const { return robot.r; }
const std::vector<Obstacle>& World::GetObstacles() const { return obs.obstacles; }
const WorldSize& World::GetWorldSize() const { return worldSize; }
