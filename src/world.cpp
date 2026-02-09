#include "world.h"






World::World(float max_x, float max_y) : worldX(max_x), worldY(max_y) {}








void World::CollisionDetection() {
	if (CollisionClose)
}







 
bool World::CollisionClose(pose& p, int threshold) {
	for (int i = 0; i < maxObs; i++) {
		int distance = (p.x - o[i].x) ^ 2 + (p.y - o[i].y) ^ 2;
		distance = distance ^ 2;
		if (distance <= threshold) {
			if (CollisionDetection(i)) {
				return true;
			}
			
		}
	}
	return false;	
}


         
