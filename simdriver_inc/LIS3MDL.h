#ifndef LIS3MDL_H
#define LIS3MDL_H

// =============================================================================
// LIS3MDL.h — Sim stub for the Pololu LIS3MDL magnetometer library
//
// Mirrors the full public API of pololu/LIS3MDL so firmware code compiles
// unchanged under SIM. No I2C bus exists in sim; read() synthesises m.x/m.y
// from ArduinoCompat::g_compassHeading (degrees 0-360, set by the simulator).
// All configuration methods are silent no-ops.
// =============================================================================

#include <cstdint>
#include <cmath>
#include "inocompat.h"

using TwoWire = _WireClass;

class LIS3MDL {
public:

    // -------------------------------------------------------------------------
    // Nested vector type — matches the real library's template struct
    // -------------------------------------------------------------------------
    template <typename T>
    struct vector {
        T x = 0, y = 0, z = 0;
    };

    // -------------------------------------------------------------------------
    // Enumerations
    // -------------------------------------------------------------------------
    enum deviceType { device_LIS3MDL, device_auto };
    enum sa1State   { sa1_low, sa1_high, sa1_auto };

    enum regAddr {
        WHO_AM_I  = 0x0F,
        CTRL_REG1 = 0x20,
        CTRL_REG2 = 0x21,
        CTRL_REG3 = 0x22,
        CTRL_REG4 = 0x23,
        CTRL_REG5 = 0x24,
        STATUS_REG = 0x27,
        OUT_X_L   = 0x28,
        OUT_X_H   = 0x29,
        OUT_Y_L   = 0x2A,
        OUT_Y_H   = 0x2B,
        OUT_Z_L   = 0x2C,
        OUT_Z_H   = 0x2D,
        TEMP_OUT_L = 0x2E,
        TEMP_OUT_H = 0x2F,
        INT_CFG   = 0x30,
        INT_SRC   = 0x31,
        INT_THS_L = 0x32,
        INT_THS_H = 0x33,
    };

    // -------------------------------------------------------------------------
    // Public data members
    // -------------------------------------------------------------------------
    vector<int16_t> m;           // magnetometer readings
    uint8_t         last_status = 0;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------
    LIS3MDL() = default;

    // -------------------------------------------------------------------------
    // Initialisation
    // -------------------------------------------------------------------------
    bool       init(deviceType device = device_auto, sa1State sa1 = sa1_auto)
                   { _device = device_LIS3MDL; return true; }
    deviceType getDeviceType() { return _device; }
    void       enableDefault() {}

    // -------------------------------------------------------------------------
    // Register access — no-ops in sim
    // -------------------------------------------------------------------------
    void    writeReg(uint8_t /*reg*/, uint8_t /*value*/) {}
    uint8_t readReg (uint8_t /*reg*/)                    { return 0; }

    // -------------------------------------------------------------------------
    // Sensor read
    // m.x/m.y are synthesised from g_compassHeading so any code that converts
    // raw counts to a heading angle gets a sensible result. m.z stays zero.
    // -------------------------------------------------------------------------
    void read() {
        float deg = ArduinoCompat::g_compassHeading
                        ? *ArduinoCompat::g_compassHeading
                        : 0.0f;
        float rad = deg * (3.14159265f / 180.0f);
        m.x = (int16_t)( cosf(rad) * 1000.0f);
        m.y = (int16_t)( sinf(rad) * 1000.0f);
        m.z = 0;
    }

    // -------------------------------------------------------------------------
    // Static vector math helpers (match real library signatures)
    // -------------------------------------------------------------------------
    template <typename Ta, typename Tb, typename To>
    static void vector_cross(const vector<Ta>* a, const vector<Tb>* b, vector<To>* out) {
        out->x = (To)((a->y * b->z) - (a->z * b->y));
        out->y = (To)((a->z * b->x) - (a->x * b->z));
        out->z = (To)((a->x * b->y) - (a->y * b->x));
    }

    template <typename Ta, typename Tb>
    static float vector_dot(const vector<Ta>* a, const vector<Tb>* b) {
        return (float)(a->x * b->x) + (float)(a->y * b->y) + (float)(a->z * b->z);
    }

    static void vector_normalize(vector<float>* a) {
        float mag = sqrtf(a->x * a->x + a->y * a->y + a->z * a->z);
        if (mag > 0.0f) { a->x /= mag; a->y /= mag; a->z /= mag; }
    }

private:
    deviceType _device  = device_LIS3MDL;
    uint8_t    _address = 0;

    int16_t testReg(uint8_t /*address*/, regAddr /*reg*/) { return 0; }
};

#endif
