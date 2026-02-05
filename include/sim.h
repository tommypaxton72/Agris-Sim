#include <iostream>
#include "robot.h"


struct Obstacle {
	float x;
	float y;
	float radius;
};

struct Physics
{
	float friction;
}
	
class World {
public:
	World();
	init();
	Robot robo;
	obstacle o1;
	obstacle o2;
	obstacle o3;
	bool collison();
	

private:

}
