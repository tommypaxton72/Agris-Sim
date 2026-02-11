#include "robot.h"






// Robot constructor takes a robo struct and starting pose
Robot::Robot(robo config, float startX, float startY, float startTheta)
    : r(config),  // const members initialized here in the initializer list
      p({startX, startY, startTheta})
{}





// Same idea but for different outputs
void Robot::SticktoVel(float leftStick, float rightStick) {
    leftVel  = (leftStick  / 100.0f) * r.maxV;
    rightVel = (rightStick / 100.0f) * r.maxV;
}   


// Maps 0 to max speed of robot to 0 - 255 pwm?
void Robot::PWMtoVel(int leftPWM, MotorDirection leftDirection, int rightPWM, MotorDirection rightDirection) {
    leftVel = (leftPWM / 255.0f) * r.maxV;
    rightVel = (rightPWM / 255.0f) * r.maxV;
	// I dont really like this setup might try and change it later.
    if (leftDirection == MotorDirection::REVERSE)
        leftVel = -leftVel;
    if (rightDirection == MotorDirection::REVERSE)
        rightVel = -rightVel;

}




// Kinematic model
// dx/dt = velocity*cos(heading)
// dx = velocity*cos(heading)*dt
// integrate both sides
// xold + xnew = velocity * cos(heading)*(dt)


// I think something isnt right here because omega is very high when one wheel is going but slow when both are.
void Robot::KinematicUpdate() {
	vel = (rightVel + leftVel) / 2.0f;
	omega = (rightVel - leftVel) / r.wheelDistance;
}

// Overloaded function for PWMtoVel
pose Robot::UpdatePose(float dt, int leftPWM, MotorDirection leftDirection, int rightPWM, MotorDirection rightDirection) {
    pose testPose;
	PWMtoVel(leftPWM, leftDirection, rightPWM, rightDirection);
	KinematicUpdate();
	// Update pose.x
	float xDel = vel * std::cos(p.theta) * dt;
    testPose.x = p.x + xDel;
    // Update pose.y
    float yDel = vel * std::sin(p.theta) * dt;
    testPose.y = p.y + yDel;
	// Update pose.theta
    testPose.theta = p.theta + omega * dt;
	return testPose;
}

// Overloaded function for SticktoVel()
pose Robot::UpdatePose(float dt, float leftStick, float rightStick) {
    pose newPose;
    SticktoVel(leftStick, rightStick);
    KinematicUpdate();
    newPose.x     = p.x     + vel * std::cos(p.theta) * dt;
    newPose.y     = p.y     + vel * std::sin(p.theta) * dt;
    newPose.theta = p.theta + omega * dt;
    return newPose;
}

// Once a collision free pose is detected set pose
void Robot::SetPose(const pose& inPose) {
    p.x = inPose.x;
    p.y = inPose.y;
    p.theta = inPose.theta;
}    
