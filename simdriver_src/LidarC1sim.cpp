#include "LidarC1sim.h"

// =============================================================================
// LidarC1sim.cpp
// Sim implementation — data comes from ArduinoCompat::g_lidarData which
// robot.cpp fills via ray-casting before each control loop.
// No serial port or RPLidar protocol needed.
// =============================================================================

LidarC1::LidarC1()
    : serial(dummySerial) {}

LidarC1::LidarC1(HardwareSerial& serial)
    : serial(serial) {}

bool LidarC1::begin(uint32_t baudrate) { return true; }
void LidarC1::GetHealth()              {}
void LidarC1::StartScan()              { Scanning = true; }
void LidarC1::Stop()                   { Scanning = false; }
void LidarC1::Reset()                  {}
void LidarC1::MotorSpeed(uint16_t)     {}
void LidarC1::serialClear()            {}

LidarData LidarC1::GetFullScan(uint8_t NumofCycles) {
    if (ArduinoCompat::g_lidarData) {
        ArduinoCompat::g_lidarData->scanComplete = true;
        return *ArduinoCompat::g_lidarData;
    }
    return LidarData{};
}

LidarData LidarC1::GetSingleScan() {
    if (ArduinoCompat::g_lidarData && ArduinoCompat::g_lidarData->count > 0) {
        LidarData single;
        single.points[0] = ArduinoCompat::g_lidarData->points[0];
        single.count = 1;
        return single;
    }
    return LidarData{};
}

// Stubs — hardware protocol methods never called in sim
bool LidarC1::GetDescriptor(uint8_t)          { return false; }
bool LidarC1::GetHealthResponse()              { return false; }
bool LidarC1::ReadScanPacket()                 { return false; }
void LidarC1::ParseScanPacket(const uint8_t*) {}
bool LidarC1::FilterData()                     { return false; }
