#ifndef DATALAYER_H
#define DATALAYER_H

struct LidarPoint {
    float distance;
	float angle;
};

struct LidarData {
    LidarPoint[1000];
};
    
struct IMUData {
    
};

struct GPSData {
    
};

enum MotorDirection {
    FOWARD,
    REVERSE
};    

struct MotorControl {
	int PWM = 0;
	MotorDirection direction;
};

struct DataLayer {
    LidarData lidarData;
    MotorControl leftMotor;
    MotorControl rightMotor;
}    

#endif
