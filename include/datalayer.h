#ifndef DATALAYER_H
#define DATALAYER_H

struct LidarPoint {
    float distance = 0.0f;
	float angle = 0.0f;
};

struct LidarData {
    LidarPoint points[1000];
    int count = 0;
};


enum MotorDirection {
    FORWARD,
    REVERSE
};    

struct MotorControl {
	int PWM = 0;
	MotorDirection direction = FORWARD;
};

struct DataLayer {
    LidarData lidarData;
    MotorControl leftMotor;
    MotorControl rightMotor;
};

#endif
