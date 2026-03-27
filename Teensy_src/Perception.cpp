#include "Perception.h"


Perception::Perception() {}

void Perception::UpdateLidarData() {


    lidarData = lidar.GetFullScan(1);

}

void Perception::UpdateFilteredLidar() {
    
    LidarData lidar1 = lidar.GetFullScan(1);
    LidarData lidar2 = lidar.GetFullScan(1);
    // Blend Data
    LidarData blendedScan;
    blendedScan.count = lidar1.count;
    for (int i = 0; i < MAX_LIDAR_POINTS; i++) {

        if (lidar1.points[i].quality > MINIMUM_LIDAR_QUALITY && 
            lidar2.points[i].quality > MINIMUM_LIDAR_QUALITY) {
            
                blendedScan.points[i].distance = (lidar1.points[i].distance + lidar2.points[i].distance) / 2;
                blendedScan.points[i].quality = (lidar1.points[i].quality + lidar2.points[i].quality) / 2;
                blendedScan.points[i].angle = (lidar1.points[i].angle + lidar2.points[i].angle) / 2;
                blendedScan.points[i].timeStamp = (lidar1.points[i].timeStamp + lidar2.points[i].timeStamp) / 2;
                blendedScan.points[i].valid = true;

        } else if (lidar1.points[i].quality > 5) {

            blendedScan.points[i] = lidar1.points[i];

        } else if (lidar2.points[i].quality > 5) {

            blendedScan.points[i] = lidar2.points[i];

        } else {

            blendedScan.points[i].valid = false;

        }
    }
    lidarData = blendedScan;

}

// True if collision detected, false otherwise
bool Perception::CheckCollision() {
    for (int i = 0; i < MAX_LIDAR_POINTS; i++) {
        if (lidarData.points[i].distance < AGV_BUBBLE &&
            lidarData.points[i].valid) {
                return true;
            }
    }
    return false;
}

void Perception::UpdateRANSAC() {
    
    Row testRow  = ransac.RunRansac(lidarData);
    
    // Check slope difference before validating ransac line
    // Lines should not change a lot while inbetween rows
    if (testRow.leftLine.m - rowBuffer[0].leftLine.m < RANSAC_SLOPE_CHANGE_THRESHOLD) {
        rowBuffer[1].leftLine = testRow.leftLine;
        rowBuffer[1].leftLine.valid = true;
    } else {
        rowBuffer[1].leftLine.valid = false; }

    if (testRow.rightLine.m - rowBuffer[0].rightLine.m < RANSAC_SLOPE_CHANGE_THRESHOLD) {
        rowBuffer[1].rightLine = testRow.rightLine;
        rowBuffer[1].rightLine.valid = true;
    } else {
        rowBuffer[1].rightLine.valid = false;

    }
    rowBuffer[0] = rowBuffer[1];
    rowBuffer[1] = testRow;
    

}

// make waypoint if both ransac lines are done.
// If only one is valid then use that and the old one to generate a new waypoint
// or just dont make one since the old way point is still assumed valid and its far enough away to not cause issues.
void Perception::GenerateWaypoint() {
    if (rowBuffer[1].leftLine.valid && rowBuffer[1].rightLine.valid) {
    
        // Logic to generate waypoint from both lines
        float line1 = rowBuffer[1].leftLine.m * LOOKAHEAD_DISTANCE + rowBuffer[1].leftLine.b; // y = mx + b at x=0
        float line2 = rowBuffer[1].rightLine.m * LOOKAHEAD_DISTANCE + rowBuffer[1].rightLine.b; // y = mx + b at x=0
        localWaypoint.x = (line1 + line2) / 2; // Average of the two lines' intercepts
        localWaypoint.y = LOOKAHEAD_DISTANCE;
        localWaypoint.valid = true;

    } else if (rowBuffer[1].leftLine.valid) {
        
        // Logic to generate waypoint from left line and old right line
    
    
    
    } else if (rowBuffer[1].rightLine.valid) {
        // Logic to generate waypoint from right line and old left line



    } else {
        // No valid lines, return old waypoint or some default




    }
}

void Perception::Reset() {
    
}