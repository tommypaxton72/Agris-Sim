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
#include "autocontrol.h"


enum DriveMode {
    MANUAL,
    AUTO
};

// Need to go through this and private anything not needed globablly.
class Robot {
  public:
    Robot();

	void LoadConfig();

    // Changing kinematics with PWM inputs and direction    
    
    
	// Changing kinematics with controller inputs
    

    // Takes in possible pose and sets it
    void SetPose(const pose& inPose);

    void UpdateSensors(const std::vector<Obstacle>& obstacles);
	pose UpdatePose(float dt);
	void UpdateControl();

  private:
	// Struct for robot config
    robo r;
	// Struct for pose of robot
    pose p;
    DataLayer dataLayer;
	AutoControl autoControl;
    float rightVel = 0.0f;
	float leftVel = 0.0f;
    float vel = 0.0f;
    float omega = 0.0f;
    Lidar lidar;
    void KinematicUpdate();
	void SticktoVel(float leftStick, float rightStick);
    bool buttonWasPressed = 0;
    void PWMtoVel();
	Drivemode driveMode = MANUAL;
    };


#endif
