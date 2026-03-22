#include <Arduino.h>
#include "AGRIS.h"

// Deadzones and Max Speed
int LowDeadzone = 32000;
int HighDeadzone =  36000;
int MaxPWM = 127;
// Packet IDs
#define AutoDriveID 99
#define ManualDriveID 32
// Start and End Bytes
#define StartByte 56
#define EndByte 25

// Shared
AGRIS::AGRIS(){
}
// ESP32C6
#ifdef ESP32C6
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
extern XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

void AGRIS::begin(int BaudRate, int TXPin, int RXPin) {
    Serial1.begin(BaudRate, SERIAL_8N1, RXPin, TXPin);
}

void AGRIS::GetControllerData() {
    // Setting Joystick values
    // LStick
    if (xboxController.xboxNotif.joyLVert > HighDeadzone) {
        Input.LSForward = 0;
        Input.LSReverse = map(xboxController.xboxNotif.joyLVert, HighDeadzone, 65535, 0, MaxPWM);
    }
    else if (xboxController.xboxNotif.joyLVert < LowDeadzone) {
        Input.LSForward = map(xboxController.xboxNotif.joyLVert, LowDeadzone, 0, 0, MaxPWM);
        Input.LSReverse = 0;
    } 
    else {
        Input.LSForward = 0;
        Input.LSReverse = 0;
    }
    // RStick
    if (xboxController.xboxNotif.joyRVert > HighDeadzone) {
        Input.RSForward = 0;
        Input.RSReverse = map(xboxController.xboxNotif.joyRVert, HighDeadzone, 65535, 0, MaxPWM);
    }
    else if (xboxController.xboxNotif.joyRVert < LowDeadzone) {
        Input.RSForward = map(xboxController.xboxNotif.joyRVert, LowDeadzone, 0, 0, MaxPWM);
        Input.RSReverse = 0;
    } 
    else {
        Input.RSForward = 0;
        Input.RSReverse = 0;
    }
  // Turn Trigger integer values to bool values
    Input.RTrigger = xboxController.xboxNotif.trigRT > 500;
    Input.LTrigger = xboxController.xboxNotif.trigLT > 500;
    Input.Start = xboxController.xboxNotif.btnStart;
    Input.AButton = xboxController.xboxNotif.btnA;
    Input.RBumper = xboxController.xboxNotif.btnRB;
    Input.LBumper = xboxController.xboxNotif.btnLB;
}

// Dont know if i like it being a seperate function or not
// Button Debounce (Thanks CHATGPT)
void AGRIS::SetAutoDrive() {
    if (Input.Start && !AutoDriveState.OldStart) {
      AutoDriveState.AutoDrive = !AutoDriveState.AutoDrive;
    }
        AutoDriveState.OldStart = Input.Start;
}

void AGRIS::TX() {
    // AutoDrive Packet Creation
    uint8_t AutoDrivePacket[5];
    AutoDrivePacket[0] = StartByte;
    AutoDrivePacket[1] = AutoDriveID;
    AutoDrivePacket[2] = AutoDriveState.AutoDrive;
    AutoDrivePacket[3] = (AutoDrivePacket[1] + AutoDrivePacket[2]) & 0xFF;
    AutoDrivePacket[4] = EndByte;

    Serial1.write(AutoDrivePacket, sizeof(AutoDrivePacket));
    
    if (AutoDriveState.AutoDrive == false) {
        uint8_t ManualDrivePacket[13];
          ManualDrivePacket[0] = StartByte; // Start Byte (56 as Hex)
          ManualDrivePacket[1] = ManualDriveID; // Packet identification number
          // Maybe add Packet Length Byte
          ManualDrivePacket[2] = Input.RSForward; // Right Stick Forward PWM
          ManualDrivePacket[3] = Input.RSReverse; // Right Stick Reverse PWM
          ManualDrivePacket[4] = Input.LSForward; // Left Stick Forward PWM
          ManualDrivePacket[5] = Input.LSReverse; // Left Stick Reverse PWM
          ManualDrivePacket[6] = Input.RBumper; // RBumper -- Inner Right Solenoid
          ManualDrivePacket[7] = Input.LBumper; // LBumper -- Inner Left Solenoid
          ManualDrivePacket[8] = Input.RTrigger; // RTrigger -- Outer Right Solenoid
          ManualDrivePacket[9] = Input.LTrigger; // LTrigger -- Outer left Solenoid
          ManualDrivePacket[10] = Input.AButton;
          ManualDrivePacket[11] = (ManualDrivePacket[2] + ManualDrivePacket[3] + ManualDrivePacket[4] + ManualDrivePacket[5] + ManualDrivePacket[6] + ManualDrivePacket[7] + ManualDrivePacket[8] + ManualDrivePacket[9] + ManualDrivePacket[10]) & 0xff; // Check sum 
          ManualDrivePacket[12] = EndByte; // End Byte (25 as Hex)

          Serial1.write(ManualDrivePacket, sizeof(ManualDrivePacket));
    }

}
#endif











// TEENSY
#ifdef TEENSY

void AGRIS::begin(int BaudRate, int TXPin, int RXPin) {
    Serial3.begin(BaudRate);
}

void AGRIS::RX() {
    while (Serial3.available()) {
        uint8_t ReadByte = Serial3.read();
        if (ReadByte == StartByte) {
            i = 0;
            PacketLength = 0;
        }
            Buffer[i++] = ReadByte;
        if (i == 2) {
            switch (Buffer[1]) {
                case AutoDriveID: PacketLength = 5;
                break;
                case ManualDriveID: PacketLength = 13;
                break;
                // Could add more cases if we needed to
                default: 
                    i = 0;
                    PacketLength = 0;
                break;
            }   
        }
        if (PacketLength && i == PacketLength) {
            UnpackData(Buffer, PacketLength);
            i = 0;
            PacketLength = 0;
        };
    };
}

void AGRIS::UnpackData(uint8_t* Buffer, uint8_t PacketLength) {
    //Checksum
    switch (Buffer[1]) {
        case AutoDriveID: 
            CheckSum = (Buffer[1] + Buffer[2]) & 0xFF;
            if (CheckSum == Buffer[3]) {
                AutoDriveState.AutoDrive = Buffer[2];
            } else {
            #ifdef Debug
            Serial.print("AutoDrive CheckSum Error");
            #endif
            }
            break;
        case ManualDriveID:
            CheckSum = (Buffer[2] + Buffer[3] + Buffer[4] + Buffer[5] + Buffer[6] + Buffer[7] + Buffer[8] + Buffer[9] + Buffer[10]) & 0xff;
            if (CheckSum == Buffer[11]) {
                Output.RWForward = Buffer[2];
                Output.RWReverse = Buffer[3];
                Output.LWForward = Buffer[4];
                Output.LWReverse = Buffer[5];
                Output.RISolenoid = Buffer[6];
                Output.LISolenoid = Buffer[7];
                Output.ROSolenoid = Buffer[8];
                Output.LOSolenoid = Buffer[9];
                Output.Pump = Buffer[10];
            } else {
            #ifdef Debug
            Serial.print("ManualDrive CheckSum Error");
            #endif
            }
            break;
    }
}
#endif


#ifdef RPI




#endif