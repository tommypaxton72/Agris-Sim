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

/*

*/

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
        void SetEORWaypoint();
        void SetEndOfRowLine();
        void SetLidarEndOfRow(uint16_t n);
        void ClearEOR();
        void UpdateRowDistance();

        // Getters
        const LidarData& GetLidarData() const { return lidarData;};
        const LidarData& GetEndOfRowData() const { return endofRowLidar; };
        const RansacLine& GetEndOfRowLine() const { return ransacLineEOR; }
        const Row& GetRowData() const { return rowBuffer[1]; };
        const Waypoint& GetLocalWaypoint() const { return localWaypoint; };

        const bool LeftLineValid()  const { return rowBuffer[1].leftLine.valid; };
        const bool RightLineValid() const { return rowBuffer[1].rightLine.valid; };
        float GetAvgRowDistance()   const { return avgRowDistance; };

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
        LidarData endofRowLidar; // Buffer holding sorted lidar data looking for end of row
        Waypoint localWaypoint;
        Row rowBuffer[ROW_BUFFER_SIZE]; // Need to implement better way of setting row buffer.
        
        RansacLine ransacLineEOR;
        Waypoint waypointEOR;

        bool readyToTurn = false;
        float avgRowDistance = ROW_WIDTH_ESTIMATE; // Starting guess for distance between rows.
        

        


};

#endif