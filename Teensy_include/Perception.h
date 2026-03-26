#ifndef PERCEPTION_H
#define PERCEPTION_H

#ifndef SIM
#include <Arduino.h>
#include "LidarC1.h"
#endif

#ifdef SIM
#include "inocompat.h"
#include "LidarC1sim.h"
#endif

#include <cstdint>
#include "datastructs.h"
#include "Config.h"
#include "RANSAC.h"

class Perception {
    public:
        Perception();
        
        void Update();

        bool CheckCollision();
        void UpdateLidarData();
        void UpdateFilteredLidar();
        void UpdateRANSAC();
        void GenerateWaypoint();
        void Reset();

        bool LeftLineValid() { return rowBuffer[1].leftLine.valid; };
        bool RightLineValid() { return rowBuffer[1].rightLine.valid; };
        // Getters
        LidarData& GetLidarData() {return lidarData;};
        RansacLine& GetLeftRansac() {return rowBuffer[1].leftLine; };
        RansacLine& GetRightRansac() {return rowBuffer[1].rightLine; };
        Waypoint& GetLocalWaypoint() {return localWaypoint;};

        // Might look at making circular buffer for Row storage
        // uint8_t IncrementRowBuffer() { return }
    private:
        RANSAC ransac;

        LidarC1 lidar;


        LidarData lidarData;
        Waypoint localWaypoint;
        Row rowBuffer[ROW_BUFFER_SIZE];
        
        

        


};

#endif