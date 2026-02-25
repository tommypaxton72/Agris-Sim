#include "world.h"
#include <yaml-cpp/yaml.h>

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

// Updates the world
// Takes in Controller inputs
// Creates new robot pose
// Before robot pose is set it checks for collision
// Sets Pose
void World::Update(float dt) {
    
		// Update Sensors
        robot.UpdateSensors(GetObstacles());

        
		// Check pose
        pose testPose = robot.UpdatePose(dt);
		// Set Pose
        if (!CollisionDetection(testPose) && !EdgeDetection(testPose)) {
			robot.SetPose(testPose);
        }


        }


// Returns true if collision with edge is detected and false if free. 
bool World::EdgeDetection(const pose& p) {
	const robo& r = robot.GetRobo();
    if (p.x + r.width / 2 <= worldSize.x &&
        p.x + r.width / 2 >= 0 &&
        p.y + r.length / 2 <= worldSize.y &&
        p.y + r.length / 2 >= 0) {
        return false;
    }
    return true;
}

// Narrow phase collision detection using local pose transformation AABB vs Circle.
// Returns true if there is a collision and false if there isnt one.
bool World::CollisionDetection(const pose& p) {
    
    std::vector<Obstacle> nearby = CollisionClose(p);
    const robo r = robot.GetRobo();
    for (const auto& obstacle : nearby) {
		// Get vector from robot to obstacle.
		float gDistX = p.x - obstacle.x;
		float gDistY = p.y - obstacle.y;
		// Transform from global frame to robots local frame.
		point local = Transform(gDistX, gDistY, -p.theta);
		// Find closest point on robot edge to obstacle center.
		float closestX = Clamp(local.x, -1 * r.width / 2, r.width / 2);
		float closestY = Clamp(local.y, -1 * r.length / 2,r.length / 2);
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
std::vector<Obstacle> World::CollisionClose(const pose& p) {
    std::vector<Obstacle> nearby; 
	const robo& r = robot.GetRobo();
	const std::vector<Obstacle>& o = obs.GetObstacles();
    for (int i = 0; i < o.size(); i++) {
        double dx = p.x - o[i].x;
        double dy = p.y - o[i].y;
        double distSquared = dx * dx + dy * dy;
		// robot.length is used because I imagine length will usually be larger than width.
		double threshold = r.length + o[i].radius; 
        if (distSquared <= threshold * threshold) {
            nearby.push_back(o[i]);
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

void World::PrintLidarDebug() {
    const LidarData& data = robot.GetLidarData();  // need this getter on Robot
    
    // Print count so we know rays are being generated
    std::cout << "[Lidar] Count: " << data.count << "\n";
    
    // Print a sample of points rather than all 720 - every 90 degrees gives us 8 readings
    for (int i = 0; i < data.count; i += data.count / 8) {
        std::cout << "  angle: " << data.points[i].angle 
                  << "  dist: "  << data.points[i].distance << "\n";
    }
}
