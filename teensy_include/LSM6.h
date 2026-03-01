#ifndef LSM6_H
#define LSM6_H

// =============================================================================
// LSM6.h — Sim stub for the Pololu LSM6 IMU library
//
// The real library talks to the IMU chip over I2C and fills g and a
// with live gyro and accelerometer readings.
//
// In sim there is no I2C bus, so we return fake data instead.
// g.z is given a small constant value so the gyro correction term
// in PID is non-zero and the code path actually exercises.
// Set it to 0 if you want the gyro to have no effect.
// =============================================================================

#include <cstdint>
#include "inocompat.h"

struct LSM6_vector {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

class LSM6 {
public:
    LSM6_vector g;  // gyro  — sketch reads imu.g.z for yaw rate
    LSM6_vector a;  // accel — not used in sketch but present in real lib

    // On hardware this configures the chip over I2C. No-op in sim.
    bool enableDefault() { return true; }

    // On hardware this reads fresh values from the chip into g and a.
    // Here we just write a small constant into g.z so the gyro path
    // in the sketch runs rather than always seeing zero.
    void read() {
		if (ArduinoCompat::g_dataLayer) {
			g.z = (int16_t)ArduinoCompat::g_dataLayer->imu.gyroZ;
		} else {
			g.z = 0;
		}
        g.x = 0;
        g.y = 0;
        a.x = 0;
        a.y = 0;
        a.z = 0;
    }
};

#endif
