#include "robot.h"



Robot::Robot(float startX, float startY, float startTheta) {
	p.x = startX;
	p.y = startY;
	p.theta = startTheta;
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

// This doesnt check for collision...
pose Robot::UpdatePose() {
	// Update pose.x
	float xNew = vel * std::sin(p.theta) * dt;
    p.x =+ xNew;
    // Update pose.y
    float yNew = vel * std::cos(p.theta) * dt;
	p.y =+ yNew;
	// Update pose.theta
    p.theta =+ omega * dt;
	
}

