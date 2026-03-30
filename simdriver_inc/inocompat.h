#ifndef INOCOMPAT_H
#define INOCOMPAT_H

#include <cstdint>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <iostream>
#include <random>
#include <algorithm>

// Config.h must be included before dataStruct.h so pin defines are available
// when analogWrite/digitalWrite are defined below
#include "Config.h"

// ---------------------------------------------------------------------------
// Forward declare Debug so analogWrite/digitalWrite can write into it
// datastructs.h is included below after the stubs
// ---------------------------------------------------------------------------
struct Debug;
struct LidarData;

namespace ArduinoCompat {
    inline Debug*     g_dataLayer       = nullptr;   // firmware motor outputs
    inline LidarData* g_lidarData       = nullptr;   // LidarC1sim reads scan data
    inline float*     g_gyroZ           = nullptr;   // LSM6 reads gyro Z
    inline float*     g_compassHeading  = nullptr;   // QMC5883LCompass / LIS3MDL reads heading (degrees)
    inline float*     g_gpsLat          = nullptr;   // GPS latitude  (degrees, WGS-84)
    inline float*     g_gpsLon          = nullptr;   // GPS longitude (degrees, WGS-84)
    inline float*     g_simX            = nullptr;   // Sim ground-truth pose X
    inline float*     g_simY            = nullptr;   // Sim ground-truth pose Y
    inline float*     g_simTheta        = nullptr;   // Sim ground-truth pose theta

    inline void SetDataLayer(Debug* d)      { g_dataLayer      = d; }
    inline void SetLidarData(LidarData* l)  { g_lidarData      = l; }
    inline void SetGyroZ(float* g)          { g_gyroZ          = g; }
    inline void SetCompassHeading(float* h) { g_compassHeading = h; }
    inline void SetGPSLat(float* lat)       { g_gpsLat         = lat; }
    inline void SetGPSLon(float* lon)       { g_gpsLon         = lon; }
    inline void SetSimPose(float* x, float* y, float* theta) { g_simX = x; g_simY = y; g_simTheta = theta; }
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef INPUT
#define INPUT 0
#endif
#ifndef PI
#define PI 3.14159265358979323846f
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef HEX
#define HEX 16
#endif

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
inline unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    return (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
}

inline unsigned long micros() {
    static auto start = std::chrono::steady_clock::now();
    return (unsigned long)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start).count();
}

inline void delay(unsigned long ms) {}

// ---------------------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------------------

// constrain — clamps val between lo and hi inclusive
#define constrain(val, lo, hi) ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))

// clamp — same as constrain but as a typed inline function
// Use this when you need type safety or are working with floats
template<typename T>
inline T clamp(T val, T lo, T hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

// map — reranges x from [in_min, in_max] to [out_min, out_max]
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ---------------------------------------------------------------------------
// Random
// ---------------------------------------------------------------------------
namespace _rng {
    inline std::mt19937& engine() {
        static std::mt19937 e{std::random_device{}()};
        return e;
    }
}
inline long random(long max_val) {
    if (max_val <= 0) return 0;
    return std::uniform_int_distribution<long>(0, max_val - 1)(_rng::engine());
}
inline long random(long min_val, long max_val) {
    if (max_val <= min_val) return min_val;
    return std::uniform_int_distribution<long>(min_val, max_val - 1)(_rng::engine());
}

// ---------------------------------------------------------------------------
// HardwareSerial stub
// ---------------------------------------------------------------------------
class HardwareSerial {
public:
    void begin(unsigned long) {}
    void end() {}
    int  available()                   { return 0;  }
    int  read()                        { return -1; }
    void readBytes(uint8_t*, int)      {}
    void write(const uint8_t*, size_t) {}
    void write(uint8_t)                {}
    void flush()                       {}
};

// ---------------------------------------------------------------------------
// Serial — routes print/println to stdout
// ---------------------------------------------------------------------------
class _SerialClass : public HardwareSerial {
public:
    void begin(unsigned long) {}
    template<typename T> void print(const T& v)   { std::cout << v; }
    void print(uint8_t v, int base) { if (base == 16) std::cout << std::hex << (int)v << std::dec; else std::cout << (int)v; }
    void print(int v,     int base) { if (base == 16) std::cout << std::hex << v       << std::dec; else std::cout << v; }
    template<typename T> void println(const T& v) { std::cout << v << '\n'; }
    void println(uint8_t v, int base) { if (base == 16) std::cout << std::hex << (int)v << std::dec << '\n'; else std::cout << (int)v << '\n'; }
    void println(int v,     int base) { if (base == 16) std::cout << std::hex << v       << std::dec << '\n'; else std::cout << v << '\n'; }
    void println() { std::cout << '\n'; }
    template<typename... Args>
    void printf(const char* fmt, Args... args) { std::printf(fmt, args...); }
    int  available() { return 0; }
    void flush() {}
};
inline _SerialClass Serial;
inline _SerialClass Serial1;
inline _SerialClass Serial2;
inline _SerialClass Serial3;

// ---------------------------------------------------------------------------
// GPIO
// ---------------------------------------------------------------------------
inline void pinMode(int, int) {}
inline int  digitalRead(int)  { return 0; }

// analogWrite and digitalWrite route into Debug using pin defines from Config.h
// If g_dataLayer is null (e.g. during early init) these are safe no-ops
#include "datastructs.h"

inline void analogWrite(int pin, int value) {
    if (!ArduinoCompat::g_dataLayer) return;
    Debug& dl = *ArduinoCompat::g_dataLayer;
    if      (pin == LPWM) dl.motor.leftMotor.PWM  = value;
    else if (pin == RPWM) dl.motor.rightMotor.PWM = value;
}

inline void digitalWrite(int pin, int value) {
    if (!ArduinoCompat::g_dataLayer) return;
    Debug& dl = *ArduinoCompat::g_dataLayer;
    // LINB HIGH = left motor reverse, LOW = forward
    if      (pin == LINB) dl.motor.leftMotor.direction  = (value == HIGH) ? Reverse : Forward;
    // RINB HIGH = right motor reverse, LOW = forward
    else if (pin == RINB) dl.motor.rightMotor.direction = (value == HIGH) ? Reverse : Forward;
}

// ---------------------------------------------------------------------------
// Wire stub
// ---------------------------------------------------------------------------
class _WireClass {
public:
    void begin() {}
    void setClock(uint32_t) {}
};
inline _WireClass Wire;

#endif