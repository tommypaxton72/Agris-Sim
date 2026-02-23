#include "robot.h"


Robot::Robot() {}


void Robot::LoadConfig() {
    try {
        YAML::Node config = YAML::LoadFile("config/robot.yaml");

        r.width = config["robot"]["width"].as<float>();
        r.length = config["robot"]["length"].as<float>();
        r.maxV = config["robot"]["maxV"].as<float>();
		r.wheelDistance = config["robot"]["wheelDistance"].as<float>();
        
        
        p.x = config["robot"]["startX"].as<float>();
        p.y = config["robot"]["startY"].as<float>();
        p.theta = config["robot"]["startTheta"].as<float>();
        
    } catch (const YAML::BadFile& e) {
        std::cerr << "Could not load robot.yaml: " << e.what() << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing robot.yaml: " << e.what() << std::endl;
    }
}

void Robot::Update(float dt);

// Maps 0 to max speed of robot to 0 - 255 pwm?
void Robot::PWMtoVel(MotorControl lMotor, MotorControl rMotor) {
    leftVel = (lMotor.PWM / 255.0f) * r.maxV;
    rightVel = (rMotor.PWM / 255.0f) * r.maxV;
	// I dont really like this setup might try and change it later.
    if (lMotor.direction == REVERSE)
        leftVel = -leftVel;
    if (rMotor.direction == REVERSE)
        rightVel = -rightVel;
}

// Same idea but for different outputs
void Robot::SticktoVel(float leftStick, float rightStick) {
    leftVel  = (leftStick  / 100.0f) * r.maxV;
    rightVel = (rightStick / 100.0f) * r.maxV;
}


// Kinematic model
// dx/dt = velocity*cos(heading)
// dx = velocity*cos(heading)*dt
// xold + xnew = velocity * cos(heading)*(dt)
 

void Robot::KinematicUpdate() {
	vel = (rightVel + leftVel) / 2.0f;
	omega = (rightVel - leftVel) / r.wheelDistance;
}

// Overloaded function for PWMtoVel
pose Robot::UpdatePose(float dt, MotorControl lMotor, MotorControl rMotor) {
    pose testPose;

    PWMtoVel(leftMotor, rightMotor);

    KinematicUpdate();

    // Update pose.x
    testPose.x = p.x + (vel * std::cos(p.theta) * dt);
    // Update pose.y
    testPose.y = p.y + (vel * std::sin(p.theta) * dt);
	// Update pose.theta
    testPose.theta = p.theta + (omega * dt);

    return testPose;
}

// Overloaded function for SticktoVel()
pose Robot::UpdatePose(float dt, float leftStick, float rightStick) {
    pose testPose;

    SticktoVel(leftStick, rightStick);

    KinematicUpdate();

    testPose.x     = p.x     + vel * std::cos(p.theta) * dt;
    testPose.y     = p.y     + vel * std::sin(p.theta) * dt;
    testPose.theta = p.theta + omega * dt;

    return testPose;
}

// Once a collision free pose is detected set pose
void Robot::SetPose(const pose& inPose) {
    p.x = inPose.x;
    p.y = inPose.y;
    p.theta = inPose.theta;
}    
