#include "world.h"






World::World(int max_x, int max_y) : worldX(max_x), worldY(max_y) {}


void World::Update() {  
	CollisionDetection();
	Robot::UpdatePose();
	if (!CollisionDetection && !EdgeDetection()) {
		Robot::SetPose();
	}
	
}

//Check if in bounds of world


// Transform circle from world coords to robot coords
// Clamp to edge of box
// closest point test
void World::CollisionDetection() {
// Pass in all possible close obstacles  
  if (CollisionClose) {
	  
    }
}

// loosely check for any collisions that are close
// Store any close collisions into a vector
// Pass nearby vector to collision detection
std::vector<Obstacle> World::CollisionClose(pose& p, obstacles& obs, int threshold) {
	std::vector<Obstacle*> nearby;
	// loop through vector
	for (int i = 0; i < obstacles.size(); i++) {
		double dx = p.x - obs.x;
		double dy = p.y - obs.y;
		double distSquared = dx * dx + dy * dy;

		if (distSquared =< threshold) {
			nearby.pushback(*obs);
		}
	}
	return nearby;
}


         
void RotationMatrix(){
	double dx = p.x - obs.x;
	double dy = p.y - obs.y;
	double cos = std::cos(p.heading);
	double sin = std::sin(p.heading);

	// Rotation Matrix [cos, sin]
	//                 [-sin, cos]
	double p = dx * cos + dy * sin;
	double q = -dx * sin + dy *cos;
}

