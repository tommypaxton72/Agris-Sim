#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "controller.h"
#include "types.h"
#include "datalayer.h"
#include "lidar.h"
#include "inocompat.h"
#include "sketch.h"
#include "logger.h"

enum DriveMode {
    MANUAL,
    AUTO
};


class Robot {
  public:
    Robot();
	~Robot();
        
	void LoadConfig();

    // Getters
    const pose& GetPose() const { return p; };
    const robo& GetRobo() const {
        return r;
    };
	const DataLayer& GetDataLayer() const { return dataLayer; };
	const LidarData& GetLidarData() const { return dataLayer.lidarData; };
	const RANSACLine& GetRightLine() const { return dataLayer.debug.rightLine; };
    const RANSACLine& GetLeftLine()  const { return dataLayer.debug.leftLine; };
	const MotorControl& GetLeftMotor() const { return dataLayer.leftMotor; };
    const MotorControl& GetRightMotor() const { return dataLayer.rightMotor; };
	const Debug& GetDebug() const { return dataLayer.debug; };
    const float& GetLeftVel() const { return leftVel; };
	const float& GetRightVel() const { return rightVel; };
    // Takes in possible pose and sets it
    void SetPose(const pose& inPose);

    void UpdateSensors(const std::vector<Obstacle>& obstacles);
    void UpdateControl();
	void UpdateLog();
    pose UpdatePose(float dt);
    // Getter Functions
  private:
	Controller controller;
	// Struct for robot config
    robo r;
	// Struct for pose of robot
    pose p;
    
    DataLayer dataLayer;
	Logger logger;
    //AutoControl autoControl;
    float rightVel = 0.0f;
	float leftVel = 0.0f;
    float vel = 0.0f;
    float omega = 0.0f;
    Lidar lidar;
    void KinematicUpdate();
	void SticktoVel(float leftStick, float rightStick);
    bool buttonWasPressed = false;
	bool keyWasPressed = false;
    void PWMtoVel();
	DriveMode driveMode = MANUAL;
    };


#endif
