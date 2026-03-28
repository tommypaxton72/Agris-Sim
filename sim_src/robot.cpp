#include "robot.h"


Robot::Robot() : lidar(720, 20.0f) {
}

Robot::~Robot() {
    logger.Close();
}

void Robot::LoadConfig() {
    try {
        YAML::Node config = YAML::LoadFile("config/robot.yaml");
        r.width         = config["robot"]["width"].as<float>();
        r.length        = config["robot"]["length"].as<float>();
        r.maxV          = config["robot"]["maxV"].as<float>();
        r.wheelDistance = config["robot"]["wheelDistance"].as<float>();

        p.x     = config["robot"]["startX"].as<float>();
        p.y     = config["robot"]["startY"].as<float>();
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

    ArduinoCompat::SetDataLayer(&debug);
    ArduinoCompat::SetLidarData(&lidarData);
    ArduinoCompat::SetGyroZ(&gyroZ);
    setup();
    ArduinoCompat::SetDataLayer(nullptr);
    ArduinoCompat::SetLidarData(nullptr);
    ArduinoCompat::SetGyroZ(nullptr);

    logger.OpenFile("log.csv");
}

void Robot::UpdateSensors(const std::vector<Obstacle>& obstacles) {
    lidarData.scanComplete = false;
    lidarData = lidar.GetScan(p, obstacles);
}

void Robot::UpdateControl() {
    ArduinoCompat::SetDataLayer(&debug);
    ArduinoCompat::SetLidarData(&lidarData);
    ArduinoCompat::SetGyroZ(&gyroZ);

    while (!lidarData.scanComplete) {
        loop();
    }

    ArduinoCompat::SetDataLayer(nullptr);
    ArduinoCompat::SetLidarData(nullptr);
    ArduinoCompat::SetGyroZ(nullptr);
}

void Robot::UpdateLog() {
    if (driveMode == AUTO) {
        logger.LogData(p, debug);
    }
}

void Robot::PWMtoVel() {
    leftVel  = (debug.motor.leftMotor.PWM  / 255.0f) * r.maxV;
    rightVel = (debug.motor.rightMotor.PWM / 255.0f) * r.maxV;
    if (debug.motor.leftMotor.direction  == Reverse) leftVel  = -leftVel;
    if (debug.motor.rightMotor.direction == Reverse) rightVel = -rightVel;
}

void Robot::SticktoVel(float leftStick, float rightStick) {
    leftVel  = (leftStick  / 100.0f) * r.maxV;
    rightVel = (rightStick / 100.0f) * r.maxV;
}

void Robot::KinematicUpdate() {
    vel   = (rightVel + leftVel)  / 2.0f;
    omega = (rightVel - leftVel) / r.wheelDistance;
    if (debug.motor.leftMotor.PWM > 0 || debug.motor.rightMotor.PWM > 0) {
        gyroZ = (omega * (180.0f / M_PI)) / 0.070f;
    }
}

pose Robot::UpdatePose(float dt) {
    controller.Update();

    bool buttonIsPressed   = controller.GetButton();
    bool buttonJustPressed = buttonIsPressed && !buttonWasPressed;

    bool keyIsPressed   = sf::Keyboard::isKeyPressed(sf::Keyboard::Space);
    bool keyJustPressed = keyIsPressed && !keyWasPressed;

    if (buttonJustPressed || keyJustPressed) {
        driveMode = (driveMode == MANUAL) ? AUTO : MANUAL;
        std::cout << "[Robot] DriveMode: " << (driveMode == AUTO ? "AUTO" : "MANUAL") << "\n";
    }
    buttonWasPressed = buttonIsPressed;
    keyWasPressed    = keyIsPressed;

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
    testPose.x     = p.x + (vel * std::cos(p.theta) * dt);
    testPose.y     = p.y + (vel * std::sin(p.theta) * dt);
    testPose.theta = p.theta + (omega * dt);
    return testPose;
}

void Robot::SetPose(const pose& inPose) {
    p.x     = inPose.x;
    p.y     = inPose.y;
    p.theta = inPose.theta;
}
