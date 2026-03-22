#include "MotorControl.h"

// Notes
// How to incorporate a PID loop for controlling wheel speed and turn rate?


void MotorController::Forward(uint8_t speed) {
    analogWrite(LPWM, speed);
    analogWrite(RPWM, speed);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, LOW);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, LOW);
}

void MotorController::Reverse(uint8_t speed) {
    analogWrite(LPWM, speed);
    analogWrite(RPWM, speed);
    digitalWrite(LINA, LOW);
    digitalWrite(LINB, HIGH);
    digitalWrite(RINA, LOW);
    digitalWrite(RINB, HIGH);
}

void MotorController::Stop() {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, HIGH);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, HIGH);
}

void MotorController::LeftTurn(uint8_t speed, int8_t intensity) {
    uint8_t leftPWM = clamp(speed - intensity, 0, 255);
    uint8_t rightPWM = clamp(speed + intensity, 0, 255);
    analogWrite(LPWM, leftPWM);
    analogWrite(RPWM, rightPWM);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, LOW);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, LOW);
}

void MotorController::RightTurn(uint8_t speed, int8_t intensity) {
    uint8_t leftPWM = clamp(speed + intensity, 0, 255);
    uint8_t rightPWM = clamp(speed - intensity, 0, 255);
    analogWrite(LPWM, leftPWM);
    analogWrite(RPWM, rightPWM);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, LOW);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, LOW);
}

void MotorController::AutoForward(uint8_t leftPWM, uint8_t rightPWM) {
    analogWrite(LPWM, leftPWM);
    analogWrite(RPWM, rightPWM);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, LOW);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, LOW);
}
void MotorController::AutoReverse(uint8_t leftPWM, uint8_t rightPWM) {
    analogWrite(LPWM, leftPWM);
    analogWrite(RPWM, rightPWM);
    digitalWrite(LINA, LOW);
    digitalWrite(LINB, HIGH);
    digitalWrite(RINA, LOW);
    digitalWrite(RINB, HIGH);
}
