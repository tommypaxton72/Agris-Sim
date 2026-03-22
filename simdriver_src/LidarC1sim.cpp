#include "LidarC1sim.h"

// =============================================================================
// LidarC1_sim.cpp
// Sim implementation — all data comes from DataLayer via RPLidarC1sim.
// No serial port is used. Drop this file into simdriver_src/ so CMake
// picks it up for sim builds and ignores LidarC1_hw.cpp.
// =============================================================================
LidarC1::LidarC1()
    : serial(dummySerial), simLidar(nullptr) {}
    
LidarC1::LidarC1(HardwareSerial& serial) 
    : serial(serial), simLidar(nullptr) {}

bool LidarC1::begin(uint32_t baudrate) {
    // RPLidarC1sim::begin detects g_dataLayer and skips hardware init
    return simLidar.begin(baudrate);
}

void LidarC1::GetHealth() {
    RPLidarHealth h;
    simLidar.get_health(&h);
    Health.status    = h.status;
    Health.errorCode = h.error_code;
}

void LidarC1::StartScan() {
    simLidar.start_scan();
    Scanning = true;
}

void LidarC1::Stop() {
    simLidar.stop_scan();
    Scanning = false;
}

// No-ops in sim — no motor or serial to control
void LidarC1::Reset()               {}
void LidarC1::MotorSpeed(uint16_t)  {}
void LidarC1::serialClear()         {}

LidarData LidarC1::GetFullScan(uint8_t NumofCycles) {
    LidarData scan;
    RPLidarMeasurement m;
    uint16_t IDX        = 0;
    uint8_t  cyclesDone = 0;

    while (cyclesDone < NumofCycles && IDX < MAX_LIDAR_POINTS) {
        if (!simLidar.get_measurement(&m)) break;

        // start_flag marks the wrap point between rotations
        if (m.start_flag && IDX > 0) {
            cyclesDone++;
            if (cyclesDone >= NumofCycles) break;
        }

        scan.points[IDX].angle     = m.angle;
        scan.points[IDX].distance  = m.distance;
        scan.points[IDX].quality   = m.quality;
        scan.points[IDX].valid     = (m.distance > 0.0f);
        scan.points[IDX].timeStamp = (uint16_t)(m.timestamp & 0xFFFF);
        IDX++;
    }

    scan.count = IDX;
    // In LidarC1sim::GetFullScan()
    Serial.print("[LidarC1sim] scan.count=");
    Serial.print(IDX);
    Serial.print(" sample angle=");
    Serial.print(scan.points[0].angle);
    Serial.print(" dist=");
    Serial.print(scan.points[0].distance);
    Serial.print(" quality=");
    Serial.println((int)scan.points[0].quality);

    // Unblock robot.cpp's while(!scanComplete) loop
    if (ArduinoCompat::g_dataLayer) {
        ArduinoCompat::g_dataLayer->lidarData.scanComplete = true;
    }

    return scan;
}

LidarData LidarC1::GetSingleScan() {
    LidarData scan;
    RPLidarMeasurement m;
    if (simLidar.get_measurement(&m)) {
        scan.points[0].angle     = m.angle;
        scan.points[0].distance  = m.distance;
        scan.points[0].quality   = m.quality;
        scan.points[0].valid     = (m.distance > 0.0f);
        scan.points[0].timeStamp = (uint16_t)(m.timestamp & 0xFFFF);
        scan.count               = 1;
    }
    return scan;
}

// Stubs — hardware protocol methods never called in sim
bool LidarC1::GetDescriptor(uint8_t)          { return false; }
bool LidarC1::GetHealthResponse()              { return false; }
bool LidarC1::ReadScanPacket()                 { return false; }
void LidarC1::ParseScanPacket(const uint8_t*) {}
bool LidarC1::FilterData()                     { return false; }