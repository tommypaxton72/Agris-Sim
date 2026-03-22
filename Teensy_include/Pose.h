#ifndef POSE_H
#define POSE_H

#ifndef SIM
#include <Arduino.h>
#include "LSM6.h"
#include "QMC5883LCompass.h"
#endif

#ifdef SIM
#include "inocompat.h"
#endif

#include <cstdint>
#include "datastructs.h"

class Position {
    public:
        Position();

        // Call every loop — reads all sensors and updates pose
        void UpdatePose();

        // Getters
        const Pose& GetCurrentPose() { return currentPose; };

    private:
        Pose currentPose;

        // --- Sensor objects ---
        #ifndef SIM
        LSM6 imu;
        QMC5883LCompass compass;
        #endif

        // --- Timing ---
        uint32_t lastUpdateTime = 0; // tracks dt between updates

        // --- Encoder state ---
        // Raw tick counts read from hardware each loop
        int32_t leftTicks  = 0;
        int32_t rightTicks = 0;
        int32_t lastLeftTicks  = 0;
        int32_t lastRightTicks = 0;

        // --- GPS state ---
        float gpsX = 0.0f;
        float gpsY = 0.0f;
        bool  gpsNewFix = false; // true when a fresh GPS fix is available

        // --- Heading complementary filter state ---
        float filteredHeading = 0.0f; // radians, 0 = forward

        // --- Sensor update methods ---
        void ReadIMU();        // reads gyro Z rate
        void ReadCompass();    // reads absolute heading
        void ReadEncoders();   // reads left and right tick counts
        void ReadGPS();        // reads GPS position if new fix available

        // --- Fusion methods ---
        void UpdateHeading(float dt);   // complementary filter: gyro + compass
        void UpdatePosition(float dt);  // dead reckoning from encoders + GPS snap

        // --- Helpers ---
        float TicksToMM(int32_t ticks); // converts encoder ticks to mm traveled
};

#endif
