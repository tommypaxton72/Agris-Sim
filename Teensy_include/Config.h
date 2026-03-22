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
#define MAXCYCLES 1
#define MAX_LIDAR_POINTS 1000
#define LIDAR_MIN_QUALITY 2

// RANSAC
#define MIN_RIGHT_ANGLE 0
#define MAX_RIGHT_ANGLE 180
#define MIN_LEFT_ANGLE 181
#define MAX_LEFT_ANGLE 360
#define RANSAC_DISTANCE_THRESHOLD 50
#define RANSAC_DISTANCE_THRESHOLD_SQ (RANSAC_DISTANCE_THRESHOLD * RANSAC_DISTANCE_THRESHOLD)
#define RANSAC_MAX_ITERATIONS 360
#define NUM_RANSAC_LINES 50

// PERCEPTION
#define LOOKAHEAD_DISTANCE 500 // in mm
#define AGV_BUBBLE 255 // in mm
#define ROW_BUFFER_SIZE 2
#define RANSAC_SLOPE_CHANGE_THRESHOLD 0.5f
#define MINIMUM_LIDAR_QUALITY 2
#define MAX_WAYPOINTS 5

// CONTROL
#define WHEELBASE_MM 279.4f   // 11 inches center to center in mm
#define CRUISE_SPEED 120

//STATE MACHINE
#define STATEDETECTION_DELAY 250
#define WAYPOINT_TIMER 1000



#endif
