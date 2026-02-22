#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "controller.h"


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

enum MotorDirection {
    FOWARD,
    REVERSE
};    

struct MotorControl {
	int PWM = 0;
	MotorDirection direction;
};    



// Need to go through this and private anything not needed globablly.
class Robot {
  public:
    Robot();
	// Load config from YAML file
    void LoadConfig();

    //Create Controller object
    Controller controller;

	// Struct for robot config
    robo r;
	// Struct for pose of robot
    pose p;

	// Structs for motor controller
    MotorControl leftMotor;
	MotorControl rightMotor;

    // Changing kinematics with PWM inputs and direction    
    void PWMtoVel(MotorControl lMotor, MotorControl rMotor);
    pose UpdatePose(MotorControl lMotor, MotorControl rMotor);

	// Changing kinematics with controller inputs
	void SticktoVel(float leftStick, float rightStick);
    pose UpdatePose(float dt, float leftStick, float rightStick);
    
	// Based on differential drive system
    void KinematicUpdate();

	// Takes in possible pose and sets it
    void SetPose(const pose& inPose);

  private:  
	float rightVel = 0.0f;
	float leftVel = 0.0f;
    float vel = 0.0f;
    float omega = 0.0f;
	
};


#endif
