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

/*

*/

class Position {
    public:
        Position();

        // Call every loop — reads all sensors and updates pose
        void UpdatePose();
        void SetEORPose();
        // Getters
        const Pose& GetCurrentPose() const { return currentPose; };
        Waypoint GetEORWaypoint();
    private:
        Pose currentPose;
        Pose poseEOR;


        #ifndef SIM
        LSM6 imu;
        QMC5883LCompass compass;
        #endif


        uint32_t lastUpdateTime = 0;



        int32_t leftTicks  = 0;
        int32_t rightTicks = 0;
        int32_t lastLeftTicks  = 0;
        int32_t lastRightTicks = 0;


        float gpsX = 0.0f;
        float gpsY = 0.0f;
        bool  gpsNewFix = false;

        float filteredHeading = 0.0f;


        void ReadIMU();
        void ReadCompass();
        void ReadEncoders();
        void ReadGPS();


        void UpdateHeading(float dt);
        void UpdatePosition(float dt);


        float TicksToMM(int32_t ticks);
        
        

};

#endif
