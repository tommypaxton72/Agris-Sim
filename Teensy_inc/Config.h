#ifndef CONFIG_H_
#define CONFIG_H_

// PINS
#define LPWM 3
#define LINA 5
#define LINB 6
#define RPWM 4
#define RINA 7
#define RINB 8

#define LInnerSolenoid 9
#define LOuterSolenoid 10
#define RInnerSolenoid 11
#define ROuterSolenoid 12
#define OutPump 13

//LIDAR
#define MAXCYCLES 1 // Max number of cycles stored 
#define MAX_LIDAR_POINTS 1000
#define LIDAR_MIN_QUALITY 2 // Same as below. Need to consolidate this

// RANSAC
#define MIN_RIGHT_ANGLE 205
#define MAX_RIGHT_ANGLE 335
#define MIN_LEFT_ANGLE 25
#define MAX_LEFT_ANGLE 155
#define RANSAC_DISTANCE_THRESHOLD 50 // Maximum distance a point can be to be declared an inlier
#define RANSAC_DISTANCE_THRESHOLD_SQ (RANSAC_DISTANCE_THRESHOLD * RANSAC_DISTANCE_THRESHOLD)
#define RANSAC_MAX_ITERATIONS 360 // How many times ransac runs before it kicks out
#define NUM_RANSAC_LINES 50

// PERCEPTION
#define LOOKAHEAD_DISTANCE 750 // How far waypoints are generated ahead of RANSAC line center
#define AGV_BUBBLE 350 // Collision detection threshold distance
#define ROW_BUFFER_SIZE 2 // Not really implemented yet but evenutally for a circular buffer for RANSAC lines
#define RANSAC_SLOPE_CHANGE_THRESHOLD 0.5f // Thresholdo of slope change for ransac lines to stay valid
#define MINIMUM_LIDAR_QUALITY 2 // Minimum lidar quality that is accepted
#define MAX_WAYPOINTS 5 // Max number of global stored waypoints
#define NUM_OF_HITS 10 // Number of hits before a row is not detected

// CONTROL
#define WHEELBASE_MM 279.4f   // Wheel to Wheel distance for pure pursuit calculations
#define CRUISE_SPEED 120 // PWM for straight forward speed

//STATE MACHINE
#define STATEDETECTION_DELAY 250 // Time between StateDetection runs
#define WAYPOINT_TIMER 1000 // Time between stored global waypoints.



#endif
