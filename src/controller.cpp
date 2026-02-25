#include "controller.h"
#include <iostream>

Controller::Controller() {}

void Controller::LoadConfig(const std::string& configPath) {
    try {
        YAML::Node node = YAML::LoadFile(configPath);
        config.joystickIndex = node["controller"]["joystickIndex"].as<int>();
        config.leftStickAxis = node["controller"]["leftStickAxis"].as<int>();
        config.rightStickAxis = node["controller"]["rightStickAxis"].as<int>();
        config.deadzone      = node["controller"]["deadzone"].as<float>();
        config.invertLeft    = node["controller"]["invertLeft"].as<bool>();
        config.invertRight   = node["controller"]["invertRight"].as<bool>();
    } catch (const YAML::BadFile& e) {
        std::cerr << "Could not load controller config: " << e.what() << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing controller config: " << e.what() << std::endl;
    }
}

bool Controller::IsConnected() const {
    return sf::Joystick::isConnected(config.joystickIndex);
}

float Controller::ApplyDeadzone(float value) const {
    // If stick is within deadzone range treat it as 0
    if (value > -config.deadzone && value < config.deadzone) {
        return 0.0f;
    }
    return value;
}

void Controller::Update() {
    if (!IsConnected()) {
                // W/S controls left stick, Up/Down controls right stick
        leftStick  = 0.0f;
        rightStick = 0.0f;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) leftStick  =  100.0f;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) leftStick  = -100.0f;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))   rightStick =  100.0f;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) rightStick = -100.0f;
        return;
    }

    // Update SFML joystick state before reading
    sf::Joystick::update();

    // Read raw axis values
    float rawLeft  = sf::Joystick::getAxisPosition(
                        config.joystickIndex, 
                        static_cast<sf::Joystick::Axis>(config.leftStickAxis));
    float rawRight = sf::Joystick::getAxisPosition(
                        config.joystickIndex, 
                        static_cast<sf::Joystick::Axis>(config.rightStickAxis));

    // Apply deadzone then invert if configured
    leftStick  = ApplyDeadzone(rawLeft)  * (config.invertLeft  ? -1.0f : 1.0f);
    rightStick = ApplyDeadzone(rawRight) * (config.invertRight ? -1.0f : 1.0f);
}

bool Controller::GetButton() const {
    return sf::joystick::isButtonPressed(config.joystickIndex, 1);
}    

float Controller::GetLeftStick() const { return leftStick; }
float Controller::GetRightStick() const { return rightStick; }
