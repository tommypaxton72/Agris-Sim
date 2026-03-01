#ifndef DATALAYER_H
#define DATALAYER_H

struct LidarPoint {
    float distance = 0.0f;
	float angle = 0.0f;
};

struct LidarData {
    LidarPoint points[1000];
    int count = 0;
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
    float a     = 0.0f;
    float b     = 0.0f;
    float c     = 0.0f;
    bool  valid = false;  // only draw if RANSAC found a line with enough inliers
};

struct Debug {
    float lineDifference;
    RANSACLine leftLine;
	RANSACLine rightLine;
    float zRate;
    float PIDResult;
    float leftDistance = 0.0f;
    float rightDistance = 0.0f;
	int state = 0;
    };    

struct DataLayer {
    LidarData lidarData;
	IMU imu;
    MotorControl leftMotor;
    MotorControl rightMotor;
	Debug debug;
    };

#endif
