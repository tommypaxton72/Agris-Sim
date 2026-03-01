#include "robot.h"


Robot::Robot() : lidar(720, 20.0f) {
}


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

        int rays   = config["robot"]["lidarRays"].as<int>();
        float maxD = config["robot"]["lidarMaxDistance"].as<float>();
        lidar.SetConfig(rays, maxD);

    } catch (const YAML::BadFile& e) {
        std::cerr << "Could not load robot.yaml: " << e.what() << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing robot.yaml: " << e.what() << std::endl;
    }
    controller.LoadConfig("config/sim.yaml");
	ArduinoCompat::SetDataLayer(&dataLayer);
	setup();
	ArduinoCompat::SetDataLayer(nullptr);
}

void Robot::UpdateSensors(const std::vector<Obstacle>& obstacles) {
    // Pass obstacles to lidar to generate a fresh scan this frame
    dataLayer.lidarData = lidar.GetScan(p, obstacles);
}

void Robot::UpdateControl() {

    ArduinoCompat::SetDataLayer(&dataLayer);

    const float msPerPoint = 100.0f / dataLayer.lidarData.count;
    unsigned long timeStep = 0;
    while (!dataLayer.lidarData.scanComplete) {
        ArduinoCompat::simTimeOffset = (unsigned long)timeStep;
        timeStep += msPerPoint;
        loop();
    }
	ArduinoCompat::simTimeOffset = (unsigned long)timeStep;

    ArduinoCompat::SetDataLayer(nullptr);
}



// Maps 0 to max speed of robot to 0 - 255 pwm?
void Robot::PWMtoVel() {
    leftVel = (dataLayer.leftMotor.PWM / 255.0f) * r.maxV;
    rightVel = (dataLayer.rightMotor.PWM / 255.0f) * r.maxV;
	// I dont really like this setup might try and change it later.
    if (dataLayer.leftMotor.direction == REVERSE) leftVel = -leftVel;
    if (dataLayer.rightMotor.direction == REVERSE) rightVel = -rightVel;
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
    if (dataLayer.leftMotor.PWM > 0 || dataLayer.rightMotor.PWM > 0) {
        dataLayer.imu.gyroZ = (omega * (180.0f / M_PI)) / 0.070f;
    }
}    
pose Robot::UpdatePose(float dt) {
    controller.Update();

    //
	bool buttonIsPressed = controller.GetButton();
    bool buttonJustPressed = buttonIsPressed && !buttonWasPressed;

    bool keyIsPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::Space);
	bool keyJustPressed = keyIsPressed && !keyWasPressed;
    
    if (buttonJustPressed || keyJustPressed) {
        driveMode = (driveMode == MANUAL) ? AUTO : MANUAL;
        std::cout << "[Robot] DriveMode: " << (driveMode == AUTO ? "AUTO" : "MANUAL") << "\n";
	}
    buttonWasPressed = buttonIsPressed;
    keyWasPressed = keyIsPressed;
    
    pose testPose;
    if (driveMode == AUTO) {
		UpdateControl();
		PWMtoVel();
    } else {
        float lStick = controller.GetLeftStick();
		float rStick = controller.GetRightStick();
		SticktoVel(lStick, rStick);
    }
	KinematicUpdate();
    // Update pose.x
    testPose.x = p.x + (vel * std::cos(p.theta) * dt);
    // Update pose.y
    testPose.y = p.y + (vel * std::sin(p.theta) * dt);
    // Update pose.theta
    testPose.theta = p.theta + (omega * dt);
    return testPose;
	
}


// Once a collision free pose is detected set pose
void Robot::SetPose(const pose& inPose) {
    p.x = inPose.x;
    p.y = inPose.y;
    p.theta = inPose.theta;
}



