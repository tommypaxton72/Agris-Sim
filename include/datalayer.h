#ifndef DATALAYER_H
#define DATALAYER_H

#include <cstdint>

struct LidarPoint {
    float distance = 0.0f;
	float angle = 0.0f;
    uint8_t quality = 0;
    bool valid = false;
    uint16_t timeStamp = 0;
};

struct LidarData {
    LidarPoint points[1000];
    int count = 0;
    uint32_t time = 0;
	bool scanComplete = false;
};

struct IMU {
    float gyroZ = 0.0f;
};    

enum MotorDirection {
    FORWARD,
    REVERSE
};    

struct MotorControl {
	int PWM = 0;
	MotorDirection direction = FORWARD;
};

struct RANSACLine {
    float m = 0; // slope
    float b = 0; // intercept
    uint16_t inliers = 0; // number of inliers
    bool valid = false;
};

struct PIDControl {
    float Kp = 0.35f;
    float Ki = 0.0f;
    float Kd = 0.05f;
};

struct DriveControl {
    int baseSpeed = 100;
    float steeringLimitRatio = 0.8f;
    int minMotorPWM = 40;
    int maxMotorPWM = 255;
    float aggressiveThreshold = 200.0f;
    float aggressiveMultiplier = 1.5f;
};    

struct DebugWaypoint {
    float x = 0;
    float y = 0;
};

struct Debug {
    float lineDifference  = 0.0f;
    RANSACLine leftLine;
    bool  leftValid       = false;
    RANSACLine rightLine;
    bool  rightValid      = false;
    float zRate           = 0.0f;
    float PIDResult       = 0.0f;
    float leftDistance    = 0.0f;
    float rightDistance   = 0.0f;
    int   state           = 0;
    DebugWaypoint waypoint = {0, 0};
};    

struct DataLayer {
    LidarData lidarData;
	IMU imu;
    MotorControl leftMotor;
    MotorControl rightMotor;
    PIDControl PIDconfig;
	DriveControl motorConfig;
	Debug debug;
    };

#endif
