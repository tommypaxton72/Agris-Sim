#ifndef DATASTRUCTS_H
#define DATASTRUCTS_H

#ifndef SIM
#include <Arduino.h>
#endif

#ifdef SIM
#include "inocompat.h"
#endif

#include <cstdint>
#include "Config.h"

#ifndef SIM
// Lidar
struct LidarPoint {
    uint16_t timeStamp = 0;
    uint16_t distance = 0;
    uint8_t quality = 0;
    float angle = 0.0f;
    bool valid = false;
};

struct LidarData {
    LidarPoint points[MAX_LIDAR_POINTS];
    uint32_t time;
    uint16_t count;
};
#endif

// RANSAC
struct Point {
    float x;
    float y;
};

struct RansacLine {
    float m; // slope
    float b; // intercept
    uint16_t inliers; // number of inliers
    bool valid;
};


// Perception
struct Waypoint {
    float x;
    float y;
    bool valid;
};

struct Row {;
    RansacLine leftLine;
    RansacLine rightLine;
};

// Pose
struct Pose {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;;
    float velocity = 0.0f;
};




#endif