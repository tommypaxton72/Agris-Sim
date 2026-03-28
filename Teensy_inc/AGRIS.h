#ifndef AGRIS_H
#define AGRIS_H

#ifndef SIM
#include <Arduino.h>
#else
#include "inocompat.h"
#endif

#include <cstdint>
#include "Config.h"

struct AGRISController{
    uint8_t RSForward;
    uint8_t RSReverse;
    uint8_t LSForward;
    uint8_t LSReverse;
    bool RBumper;
    bool LBumper;
    bool LTrigger;
    bool RTrigger;
    bool Start;
    bool AButton;
};
struct AGRISOutput{
    uint8_t RWForward;
    uint8_t RWReverse;
    uint8_t LWForward;
    uint8_t LWReverse;
    bool RISolenoid;
    bool LISolenoid;
    bool ROSolenoid;
    bool LOSolenoid;
    bool Pump;

};
struct AGRISAutoDrive{
    bool AutoDrive;
    bool OldStart;
};

class AGRIS {
    public:
        AGRIS();
        // Functions
        void begin(int BaudRate, int TXPin, int RXPin);
        void SetAutoDrive();
        void TX();
        void RX();
        void GetControllerData();
        void UnpackData(uint8_t* Buffer, uint8_t PacketLength);
        AGRISController Input;
        AGRISAutoDrive AutoDriveState;
        AGRISOutput Output;
    private:
        uint8_t Buffer[16];
        uint8_t i;
        uint8_t PacketID;
        uint8_t PacketLength;
        uint8_t CheckSum;
};




#endif