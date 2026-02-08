#include "robot.h"



Robot::Robot(float startX, float startY, float startTheta) {
	p.x = startX;
	p.y = startY;
	p.theta = startTheta;
}











// Maps 0 to max speed of robot to 0 - 255 pwm?
void Robot::PWMtoVel(float leftPWM, float rightPWM) {
	


}




// Kinematic model
// dx/dt = velocity*cos(heading)
// dx = velocity*cos(heading)*dt
// integrate both sides
// xold + xnew = velocity * cos(heading)*(dt)

void Robot::KinematicUpdate() {
	float vel = (float rightVel + float leftVel) / 2;
	float omega = (rightVel + leftVel) / wheelDistance;
}


pose Robot::UpdatePose() {
	// Update pose.x
	float xDel = vel * std::sin(p.theta) * dt;
    float xNew =+ xDel;
    // Update pose.y
    float yDel = vel * std::cos(p.theta) * dt;
	float yNew =+ yDel;
	// Update pose.theta
    float thetaNew =+ omega * dt;
	// Before assigning new pose, make sure no collision?
}

