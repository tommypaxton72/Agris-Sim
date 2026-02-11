#include "world.h"
#include <yaml-cpp/yaml.h>







World::World(const WorldSize& size) : worldSize(size) {
    try {
        YAML::Node config = YAML::LoadFile("config/robot.yaml");
        robo robotConfig = {
            config["robot"]["width"].as<float>(),
            config["robot"]["length"].as<float>(),
            config["robot"]["maxV"].as<float>(),
            config["robot"]["wheelDistance"].as<float>()
        };
        robot = Robot(robotConfig,
            config["robot"]["startX"].as<float>(),
            config["robot"]["startY"].as<float>(),
            config["robot"]["startTheta"].as<float>()
        );
    } catch (const YAML::BadFile& e) {
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

void World::LoadControllerConfig(const std::string& configPath) {
    robot.controller.LoadConfig(configPath);
}

void World::Update(float dt) {
    // Can switch this out with different inputs
    robot.controller.Update();

    float leftStick  = robot.controller.GetLeftStick();
    float rightStick = robot.controller.GetRightStick();
	// Just make sure to change robot.UpdatePose() To accept correct inputs Its an overloaded function
    pose testPose = robot.UpdatePose(dt, leftStick, rightStick);
    if (!CollisionDetection(testPose) && !EdgeDetection(testPose)) {
        robot.SetPose(testPose);
    }
}


bool World::EdgeDetection(const pose& p) {
    
    if (p.x <= worldSize.x && p.x >= 0 &&
        p.y <= worldSize.y && p.y >= 0) {
        return false;
    }
    return true;
}

bool World::CollisionDetection(const pose& p) {
    
    std::vector<Obstacle> nearby = CollisionClose(p, obs, collisionThreshold);
    
    for (const auto& obstacle : nearby) {
		
    }
    return false;  // no collision
}

// Broad phase check - returns only obstacles within threshold distance
// Avoids running expensive collision math on every obstacle in the scene
std::vector<Obstacle> World::CollisionClose(const pose& p, const Obstacles& obs, float threshold) {
    std::vector<Obstacle> nearby;

    for (int i = 0; i < obs.obstacles.size(); i++) {
        double dx = p.x - obs.obstacles[i].x;
        double dy = p.y - obs.obstacles[i].y;
        double distSquared = dx * dx + dy * dy;

        // Compare against threshold squared to avoid costly sqrt
        // Also account for obstacle radius so we catch obstacles
        // whose edge is within threshold not just their center
        float adjustedThreshold = threshold + obs.obstacles[i].radius;
        if (distSquared <= adjustedThreshold * adjustedThreshold) {
            nearby.push_back(obs.obstacles[i]);
        }
    }
    return nearby;
}

// Getters
const pose& World::GetRobotPose() const { return robot.p; }
const robo& World::GetRobotConfig() const { return robot.r; }
const std::vector<Obstacle>& World::GetObstacles() const { return obs.obstacles; }
const WorldSize& World::GetWorldSize() const { return worldSize; }
