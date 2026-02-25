#include "autocontrol.h"


#include "autocontrol.h"
#include <iostream>
#include <cmath>
#include <algorithm>

AutoControl::AutoControl() {}

void AutoControl::Setup() {
    sm.STATE = StateMachine::STOP;
    std::cout << "[AGVController] Ready. State: STOP\n";
}

void AutoControl::Update(DataLayer& dataLayer) {
    // Point ArduinoCompat at this frame's DataLayer so analogWrite /
    // digitalWrite calls inside StateMachine land in the right place.
    ArduinoCompat::SetDataLayer(&dataLayer);

    ProcessScan(dataLayer.lidarData);

    if (leftAngles.size() > 50 || rightAngles.size() > 50)
        RunRANSAC();

    unsigned long now = millis();
    if (now - lastMotorUpdate >= 10) {
        lastMotorUpdate = now;
        RunStateMachine(dataLayer);
    }
}

void AutoControl::ProcessScan(const LidarData& data) {
    for (int i = 0; i < data.count; i++) {
        float angleDeg = data.points[i].angle * (180.0f / M_PI);
        float dist     = data.points[i].distance;

        if      (angleDeg > 45.0f  && angleDeg <= 135.0f) { rightAngles.push_back(angleDeg); rightDistances.push_back(dist); }
        else if (angleDeg > 225.0f && angleDeg <= 315.0f) { leftAngles.push_back(angleDeg);  leftDistances.push_back(dist);  }
    }

    while (rightAngles.size() > DEQUE_LIMIT) { rightAngles.pop_front(); if (!rightDistances.empty()) rightDistances.pop_front(); }
    while (leftAngles.size()  > DEQUE_LIMIT) { leftAngles.pop_front();  if (!leftDistances.empty())  leftDistances.pop_front();  }
}

void AutoControl::RunRANSAC() {
    std::vector<points> rightPts, leftPts;
    ransacRight.cartesianConversion(rightAngles, rightDistances, rightPts);
    ransacLeft.cartesianConversion(leftAngles,   leftDistances,  leftPts);

    ransacRight.RANSACLoop(rightPts);
    ransacLeft.RANSACLoop(leftPts);

    ransacRight.distancetoLine();
    ransacLeft.distancetoLine();

    rightDistance  = ransacRight.distance;
    leftDistance   = ransacLeft.distance;
    lineDifference = rightDistance - leftDistance;
}

// StateMachine methods call analogWrite/digitalWrite directly.
// ArduinoCompat intercepts those calls and writes into dataLayer.
void AutoControl::RunStateMachine(DataLayer& dataLayer) {
    bool rightOk = ransacRight.lineValidation();
    bool leftOk  = ransacLeft.lineValidation();

    switch (sm.STATE) {

        case StateMachine::STOP:
            // sm.stop() calls analogWrite(PWM1, 0) and analogWrite(PWM2, 0)
            // → intercepted into dataLayer.leftMotor.PWM and rightMotor.PWM
            sm.stop();

            if (stopStartTime == 0) stopStartTime = millis();
            if (millis() - stopStartTime > 5000) {
                if (rightOk || leftOk) {
                    sm.STATE = StateMachine::INBETWEEN_ROWS;
                    std::cout << "[SM] STOP → INBETWEEN_ROWS\n";
                }
            }
            break;

        case StateMachine::INBETWEEN_ROWS: {
            float diff = 0.0f;
            if (rightOk && leftOk) {
                diff = lineDifference;
                timeSinceValidation = 0;
            } else if (rightOk && !leftOk) {
                diff = ONE_LINE_DIST - rightDistance;
            } else if (!rightOk && leftOk) {
                diff = leftDistance - ONE_LINE_DIST;
            }

            if (!rightOk && !leftOk) {
                if (timeSinceValidation == 0) timeSinceValidation = millis();
                if (millis() - timeSinceValidation > 1500) {
                    sm.STATE = StateMachine::SEARCHING_FOR_WALLS;
                    timeSinceValidation = 0;
                    std::cout << "[SM] INBETWEEN_ROWS → SEARCHING_FOR_WALLS\n";
                    break;
                }
            }

            // sm.inbetween_rows calls:
            //   digitalWrite(INB1, HIGH) → dataLayer.leftMotor.direction  = FORWARD
            //   digitalWrite(INB2, HIGH) → dataLayer.rightMotor.direction = FORWARD
            //   analogWrite(PWM1, left)  → dataLayer.leftMotor.PWM
            //   analogWrite(PWM2, right) → dataLayer.rightMotor.PWM
            sm.inbetween_rows(diff, 0.0f);
            break;
        }

        case StateMachine::SEARCHING_FOR_WALLS:
            // sm.searching_for_walls calls analogWrite with decremented speeds
            // → intercepted into dataLayer motors
            sm.searching_for_walls();

            if (rightOk || leftOk) {
                sm.STATE = StateMachine::INBETWEEN_ROWS;
                std::cout << "[SM] SEARCHING_FOR_WALLS → INBETWEEN_ROWS\n";
            }
            break;
    }
}
