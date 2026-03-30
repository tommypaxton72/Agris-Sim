#ifndef DATASTRUCTS_H
#define DATASTRUCTS_H

#ifndef SIM
#include <Arduino.h>
#endif

#include <cstdint>
#include "Config.h"

/*

*/

enum MainState {
    AutoDrive,
    ManualDrive
};

// Do we need enum class or enum?
enum SubState {
    START,
    INBETWEEN_ROWS,
    END_OF_ROW,
    TURNING,
    ALIGNING,
    ESTOP,
    COLLISION_AVOIDANCE
};

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
    uint32_t time = 0;
    uint16_t count = 0;

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
    float z = 0.0f;     // Not really needed
    float psi = 0.0f;   // Right now everything is sorta linked to theta but might change it all to psi
    float theta = 0.0f;
    float phi = 0.0f;
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
    RansacLine lineEOR;
    Waypoint lWaypoint;
    uint8_t currentWaypointIndex;
    Waypoint gWaypoint[MAX_WAYPOINTS];
    int state = 0;
};
#endif

#endif