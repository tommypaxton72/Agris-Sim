#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <SFML/Window.hpp>
#include <string>
#include <yaml-cpp/yaml.h>


struct ControllerConfig {
    int joystickIndex;
    int leftStickAxis;
    int rightStickAxis;
    float deadzone;
    bool invertLeft;
    bool invertRight;
};

class Controller {
public:
    Controller();
    void LoadConfig(const std::string& configPath);

    // Call each frame to read current stick values
    void Update();

    // Getter Functions for stick positions
    float GetLeftStick()  const { return leftStick; }
    float GetRightStick() const { return rightStick; }

    // Check if controller is connected
    bool IsConnected() const;

private:
    ControllerConfig config;

    float leftStick  = 0.0f;
    float rightStick = 0.0f;
    
    float ApplyDeadzone(float value) const;
};
#endif
