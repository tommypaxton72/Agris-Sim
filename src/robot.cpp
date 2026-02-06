#include "robot.h"



Robot::Robot(float startX, float startY, float startTheta) {
	p.x = startX;
	p.y = startY;
	p.theta = startTheta;
}












void Robot::PWMtoVel(float leftPWM, float rightPWM) {
	


}

bool Robot::CheckCollison(float &xNew, float &yNew) {
	if (xNew == obstacle.x + obstacle.Radius || xNew == obstacle.x - obstacle.radius) {
		return false
			};
	if (yNew == obstacle.y + obstacle.radius || yNew == obstacle.y - obstacle.radius) {
		return false
			};
	return true;
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

