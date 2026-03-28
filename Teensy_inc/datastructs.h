#ifndef DATASTRUCTS_H
#define DATASTRUCTS_H

#ifndef SIM
#include <Arduino.h>
#endif

#include <cstdint>
#include "Config.h"

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

    #ifdef SIM
    bool scanComplete = false;
    #endif
};


// RANSAC
struct Point {
    float x;
    float y;
};

struct RansacLine {
    float m = 0.0f; // slope
    float b = 0.0f; // intercept
    uint16_t inliers = 0; // number of inliers
    bool valid = false;
};


// Perception
struct Waypoint {
    float x = 0.0f;
    float y = 0.0f;
    bool valid = false;
};

struct Row {
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

struct IMU {
    float x;
    float y;
    float z;
};

// MotorControl / PathFinding
enum MotorDirection {
    Forward,
    Reverse
};

struct Motor {
    uint8_t PWM = 0;
    MotorDirection direction = Forward;
};

struct MotorCommands {
    Motor leftMotor;
    Motor rightMotor;
};

#ifdef SIM
struct Debug {
    MotorCommands motor;
    Pose assumedPose;
    Row RansacLines;
    Waypoint lWaypoint;
    Waypoint gWaypoint;
    int state = 0;
};
#endif

#endif