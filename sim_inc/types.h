#ifndef TYPES_H
#define TYPES_H

#include <vector>

struct Obstacle {
    float x;
    float y;
    float radius;
};

struct robo {
	float width;
	float length;
    float maxV;
	float wheelDistance;
};

struct pose {
	float x;
	float y;
    float theta;
};

struct WorldSize {
    float x;
    float y;
};

struct point {
    float x;
    float y;
};
#endif
