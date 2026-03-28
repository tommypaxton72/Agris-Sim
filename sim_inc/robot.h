#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "controller.h"
#include "types.h"
#include "datastructs.h"
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
    const robo& GetRobo() const { return r; };
    const Debug& GetDebug() const { return debug; };
    const LidarData& GetLidarData() const { return lidarData; };
    const RansacLine& GetRightLine() const { return debug.RansacLines.rightLine; };
    const RansacLine& GetLeftLine()  const { return debug.RansacLines.leftLine; };
    const Motor& GetLeftMotor()  const { return debug.motor.leftMotor; };
    const Motor& GetRightMotor() const { return debug.motor.rightMotor; };
    const float& GetLeftVel()  const { return leftVel; };
    const float& GetRightVel() const { return rightVel; };

    void SetPose(const pose& inPose);

    void UpdateSensors(const std::vector<Obstacle>& obstacles);
    void UpdateControl();
    void UpdateLog();
    pose UpdatePose(float dt);

  private:
    Controller controller;
    robo r;
    pose p;
    Lidar lidar;
    Logger logger;

    Debug debug;
    LidarData lidarData; 
    float gyroZ = 0.0f;  

    float rightVel = 0.0f;
    float leftVel  = 0.0f;
    float vel   = 0.0f;
    float omega = 0.0f;

    void KinematicUpdate();
    void SticktoVel(float leftStick, float rightStick);
    void PWMtoVel();

    bool buttonWasPressed = false;
    bool keyWasPressed    = false;
    DriveMode driveMode   = MANUAL;
};


#endif
