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
    if (fabsf(testRow.leftLine.m - rowBuffer[0].leftLine.m) < RANSAC_SLOPE_CHANGE_THRESHOLD) {
        rowBuffer[1].leftLine = testRow.leftLine;
        rowBuffer[1].leftLine.valid = true;
    } else {
        rowBuffer[1].leftLine.valid = false; }

    if (fabsf(testRow.rightLine.m - rowBuffer[0].rightLine.m) < RANSAC_SLOPE_CHANGE_THRESHOLD) {
        rowBuffer[1].rightLine = testRow.rightLine;
        rowBuffer[1].rightLine.valid = true;
    } else {
        rowBuffer[1].rightLine.valid = false;

    }
    rowBuffer[0] = rowBuffer[1]; // Row buffer[0] = Old Data
    rowBuffer[1] = testRow;      // Row buffer[1] = New Data
    

}

// make waypoint if both ransac lines are done.
// If only one is valid then use that and the old one to generate a new waypoint
// or just dont make one since the old way point is still assumed valid and its far enough away to not cause issues.
void Perception::GenerateWaypoint() {
    if (rowBuffer[1].leftLine.valid && rowBuffer[1].rightLine.valid) {
    
        // Logic to generate waypoint from both lines
        float line1 = rowBuffer[1].leftLine.m * (-LOOKAHEAD_DISTANCE) + rowBuffer[1].leftLine.b;
        float line2 = rowBuffer[1].rightLine.m * (-LOOKAHEAD_DISTANCE) + rowBuffer[1].rightLine.b;
        localWaypoint.x = -((line1 + line2) / 2); // Negate: CW +x=rightward, PathFinder needs opposite sign
        localWaypoint.y = LOOKAHEAD_DISTANCE;
        localWaypoint.valid = true;

    } else if (rowBuffer[1].leftLine.valid) {
        localWaypoint.valid = false;
        // Logic to generate waypoint from left line and old right line
    
    
    
    } else if (rowBuffer[1].rightLine.valid) {
        // Logic to generate waypoint from right line and old left line
        localWaypoint.valid = false;


    } else {
        // No valid lines, return old waypoint or some default
        localWaypoint.valid = false;



    }
}

/* End of Row Detection
There has to be a better way that knows the end of the row.
This method lets robot keep going forward for a while before it figures it out
Maybe if there was a way for the robot to slow down as it gets closer to the end of a row?

using values as enum
0 = both rows
1 = just left
2 = just right
3 = both missing */
uint8_t Perception::CheckRows() {
    uint16_t rightHit = 0;
    uint16_t leftHit = 0;
    for (uint16_t i = 0; i < MAX_LIDAR_POINTS; i++) {
        // These angles are hard coded might change that later
        if (lidarData.points[i].angle > 90 && lidarData.points[i].angle <120 && 
            lidarData.points[i].quality > 5 && lidarData.points[i].distance < 750.0f) {
            rightHit++;
        };
        if (lidarData.points[i].angle > 270 && lidarData.points[i].angle < 300 && 
            lidarData.points[i].quality > 5 && lidarData.points[i].distance < 750.0f) {
            leftHit++;
        };
    } // These minimums are also hardcoded
    if (rightHit >= NUM_OF_HITS && leftHit >= NUM_OF_HITS) { return 0; };
    if (rightHit < NUM_OF_HITS && leftHit > NUM_OF_HITS) { return 1; };
    if (rightHit > NUM_OF_HITS && leftHit < NUM_OF_HITS) { return 2; };
    if (rightHit < NUM_OF_HITS && leftHit < NUM_OF_HITS) { return 3; };
    return 5; // return Error
}

void Perception::Reset() {
    return;
}