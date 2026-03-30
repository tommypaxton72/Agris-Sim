#ifndef QMC5883L_Compass
#define QMC5883L_Compass

// =============================================================================
// QMC5883LCompass.h — Sim stub for the mprograms/QMC5883LCompass library
//
// Mirrors the full public API of QMC5883LCompass v1.2.x so firmware code
// compiles unchanged under SIM. No I2C bus exists in sim; read() pulls the
// compass heading from ArduinoCompat::g_compassHeading (degrees 0-360, set by
// the simulator). All configuration and calibration methods are silent no-ops.
// =============================================================================

#include <cstdint>
#include <cmath>
#include "inocompat.h"

// byte is Arduino's name for uint8_t
using byte = uint8_t;

// ---------------------------------------------------------------------------
// Mode / ODR / range / OSR constants (used by setMode callers)
// ---------------------------------------------------------------------------
#define COMPASS_STANDBY    0x00
#define COMPASS_CONTINUOUS 0x01

#define COMPASS_ODR_10HZ   0x00
#define COMPASS_ODR_50HZ   0x04
#define COMPASS_ODR_100HZ  0x08
#define COMPASS_ODR_200HZ  0x0C

#define COMPASS_RNG_2G     0x00
#define COMPASS_RNG_8G     0x10

#define COMPASS_OSR_512    0x00
#define COMPASS_OSR_256    0x40
#define COMPASS_OSR_128    0x80
#define COMPASS_OSR_64     0xC0

class QMC5883LCompass {
public:

    QMC5883LCompass() = default;

    // -------------------------------------------------------------------------
    // Initialisation & configuration — all no-ops in sim
    // -------------------------------------------------------------------------
    void init() {}
    void setADDR(byte /*b*/) {}
    void setMode(byte /*mode*/, byte /*odr*/, byte /*rng*/, byte /*osr*/) {}
    void setMagneticDeclination(int /*degrees*/, uint8_t /*minutes*/) {}
    void setSmoothing(byte /*steps*/, bool /*adv*/) {}
    void setReset() {}

    // -------------------------------------------------------------------------
    // Calibration — all no-ops in sim
    // -------------------------------------------------------------------------
    void  calibrate() {}
    void  setCalibration(int /*x_min*/, int /*x_max*/,
                         int /*y_min*/, int /*y_max*/,
                         int /*z_min*/, int /*z_max*/) {}
    void  setCalibrationOffsets(float /*x*/, float /*y*/, float /*z*/) {}
    void  setCalibrationScales (float /*x*/, float /*y*/, float /*z*/) {}
    float getCalibrationOffset (uint8_t /*index*/) { return 0.0f; }
    float getCalibrationScale  (uint8_t /*index*/) { return 1.0f; }
    void  clearCalibration() {}

    // -------------------------------------------------------------------------
    // Sensor read
    // Heading is fed from ArduinoCompat::g_compassHeading (degrees 0-360).
    // Raw X/Y are synthesised so getX()/getY() return a plausible vector.
    // -------------------------------------------------------------------------
    void read() {
        float deg = ArduinoCompat::g_compassHeading
                        ? *ArduinoCompat::g_compassHeading
                        : 0.0f;
        float rad = deg * (3.14159265f / 180.0f);
        _vRaw[0] = (int)(cosf(rad) * 1000.0f);
        _vRaw[1] = (int)(sinf(rad) * 1000.0f);
        _vRaw[2] = 0;
        _azimuth  = (int)deg;
    }

    // -------------------------------------------------------------------------
    // Raw axis getters
    // -------------------------------------------------------------------------
    int getX() { return _vRaw[0]; }
    int getY() { return _vRaw[1]; }
    int getZ() { return _vRaw[2]; }

    // -------------------------------------------------------------------------
    // Azimuth — degrees 0-359 clockwise from magnetic north
    // -------------------------------------------------------------------------
    int getAzimuth() { return _azimuth; }

    // -------------------------------------------------------------------------
    // Bearing — 16-point compass index 0-15 (N=0, NNE=1, ... NNW=15)
    // -------------------------------------------------------------------------
    byte getBearing(int azimuth) {
        return (byte)(((azimuth + 11) % 360) / 22) % 16;
    }

    // -------------------------------------------------------------------------
    // Direction — writes the 3-character label from _bearings into myArray
    // (caller must provide at least 4 bytes).
    // -------------------------------------------------------------------------
    void getDirection(char* myArray, int azimuth) {
        byte idx = getBearing(azimuth);
        myArray[0] = _bearings[idx][0];
        myArray[1] = _bearings[idx][1];
        myArray[2] = _bearings[idx][2];
        myArray[3] = '\0';
    }

private:
    int  _vRaw[3]  = {0, 0, 0};
    int  _azimuth  = 0;

    const char _bearings[16][3] = {
        {' ', ' ', 'N'},
        {'N', 'N', 'E'},
        {' ', 'N', 'E'},
        {'E', 'N', 'E'},
        {' ', ' ', 'E'},
        {'E', 'S', 'E'},
        {' ', 'S', 'E'},
        {'S', 'S', 'E'},
        {' ', ' ', 'S'},
        {'S', 'S', 'W'},
        {' ', 'S', 'W'},
        {'W', 'S', 'W'},
        {' ', ' ', 'W'},
        {'W', 'N', 'W'},
        {' ', 'N', 'W'},
        {'N', 'N', 'W'},
    };
};

#endif
