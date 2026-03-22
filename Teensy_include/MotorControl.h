#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#ifndef SIM
#include <Arduino.h>
#else
#include "inocompat.h"
#endif

#include <cstdint>
#include "Config.h"

class MotorController {
   public:
        void Forward(uint8_t Speed);
        void Reverse(uint8_t Speed);
        void Stop();
        void LeftTurn(uint8_t speed, int8_t intensity);
        void RightTurn(uint8_t speed, int8_t intensity);
        void AutoForward(uint8_t leftPWM, uint8_t rightPWM);
        void AutoReverse(uint8_t leftPWM, uint8_t rightPWM);
    private:
};


#endif
