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

enum class MotorDirection {
    FORWARD,
    REVERSE
};




class Robot {
    public:
	Robot() = default;
    Robot(robo config, float startX, float startY, float startTheta);
    Controller controller;
    robo r;
    pose p;
	void PWMtoVel(int leftPWM, MotorDirection leftDirection, int rightPWM, MotorDirection rightDirection);
    pose UpdatePose(float dt,
                    int leftPWM,
                    MotorDirection leftDirection,
                    int rightPWM,
                    MotorDirection rightDirection);
	pose UpdatePose(float dt, float leftStick, float rightStick);
	void KinematicUpdate();
    void SetPose(const pose& inPose);
	void SticktoVel(float leftStick, float rightStick);
	void LoadConfig(const std::string& configPath);

  private:  
	float rightVel = 0.0f;
	float leftVel = 0.0f;
    float vel = 0.0f;
    float omega = 0.0f;
	MotorDirection leftDirection  = MotorDirection::FORWARD;
	MotorDirection rightDirection = MotorDirection::FORWARD;
};


#endif
