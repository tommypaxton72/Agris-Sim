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
        

        bool CheckCollision();
        void UpdateLidarData();
        void UpdateFilteredLidar();
        void UpdateRANSAC();
        void GenerateWaypoint();
        void Reset();
        uint8_t CheckRows();
        
        // Getters
        const LidarData& GetLidarData() const { return lidarData;};
        const Row& GetRowData() const { return rowBuffer[1]; };
        const Waypoint& GetLocalWaypoint() const { return localWaypoint; };

        const bool LeftLineValid()  const { return rowBuffer[1].leftLine.valid; };
        const bool RightLineValid() const { return rowBuffer[1].rightLine.valid; };

        // Might look at making circular buffer for Row storage
        // uint8_t IncrementRowBuffer() { return }
        #ifdef SIM
        

        #endif
    private:
        // Classes
        RANSAC ransac;
        LidarC1 lidar;

        // Structs
        LidarData lidarData;
        Waypoint localWaypoint;
        Row rowBuffer[ROW_BUFFER_SIZE]; // Need to implement better way of setting row buffer.
        
        

        


};

#endif