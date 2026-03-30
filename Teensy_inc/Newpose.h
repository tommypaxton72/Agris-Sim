#ifndef POSE_H
#define POSE_H

#ifndef SIM

#include <Arduino.h>
#else


#include "inocompat.h"
#include "LSM6.h" // IMU
#include "LIS3MDL.h" // Compass
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" // GNSS

#endif

#include <cstdint>
#include "datastructs.h"
#include "Config.h"

/*
===================== Pose ======================
Position library for AgrisNav


*/

class Position {
    public:
        Position();
        void UpdatePose();
        const Pose& GetPose() const { return currentPose; };
    private:
        void UpdateIMU();
        void UpdateCompass();
        void UpdateGPS();
        void UpdateEncoders();

        Pose currentPose;

};

#endif