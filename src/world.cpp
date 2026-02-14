#include "world.h"
#include <yaml-cpp/yaml.h>






// Most of the YAML stuff is from claude. Thanks Claude.
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
    pose testPose = robot.UpdatePose(dt, leftStick, rightStick);
	// Before updating check if collision is false
    if (!CollisionDetection(testPose) && !EdgeDetection(testPose)) {
		// Set pose
        robot.SetPose(testPose);
    }
}

// Returns true if collision with edge is detected and false if free. 
bool World::EdgeDetection(const pose& p) {

    if (p.x + robot.width / 2 <= worldSize.x &&
        p.x + robot.width / 2 >= 0 &&
        p.y + robot.width / 2 <= worldSize.y &&
        p.y + robot.width / 2 >= 0) {
        return false;
    }
    return true;
}

// Narrow phase collision detection using local pose transformation AABB vs Circle.
// Returns true if there is a collision and false if there isnt one.
bool World::CollisionDetection(const pose& p) {
    
    std::vector<Obstacle> nearby = CollisionClose(p, obs, collisionThreshold);
    
    for (const auto& obstacle : nearby) {
		
    }
    return false;
}

// Broad phase check - returns only obstacles within robot length + obstacle radius * 2
// But I might change this at some point?
std::vector<Obstacle> World::CollisionClose(const pose& p, const Obstacles& obs, float threshold) {
    std::vector<Obstacle> nearby;

    for (int i = 0; i < obs.obstacles.size(); i++) {
        double dx = p.x - obs.obstacles[i].x;
        double dy = p.y - obs.obstacles[i].y;
        double distSquared = dx * dx + dy * dy;
		// robot.length is used because I imagine length will usually be larger than width.
		double threshold = robot.length + obs.obstacles[i]; 
        if (distSquared <= threshold * threshold) {
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
