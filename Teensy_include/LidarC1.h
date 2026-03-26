#ifndef LIDARC1_h
#define LIDARC1_h

#ifndef SIM
#include <Arduino.h>
#endif

#ifdef SIM
#include "inocompat.h"  
#endif

#include "datastructs.h"
#include "Config.h"

struct ParserData{
    uint32_t timeStamp;
    bool startFlag;
    bool invFlag;
    uint8_t quality;
    float angle;
    float distance;
};

struct HealthData {
    uint8_t status;
    uint16_t errorCode;
};

class LidarC1 {
    public:
        LidarC1(HardwareSerial& serial);
        bool begin(uint32_t baudrate);
        void GetHealth();
        void StartScan();
        void Stop();
        void Reset();
        void MotorSpeed(uint16_t RPM);
        bool GetSingleScan();
        bool GetFullScan(uint8_t NumofCycles);
        void serialClear();
        HealthData Health;
    private:
        bool GetDescriptor(uint8_t expectedDataType);
        bool GetHealthResponse();
        bool ReadScanPacket();
        void ParseScanPacket(const uint8_t Packet[5]);
        ParserData Parser;

        HardwareSerial& serial;
        uint32_t serialTimeout = 1000;
        uint32_t TCMD = 0;
        bool Scanning = false;
        bool FilterData();
};



#endif
