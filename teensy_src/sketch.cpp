#include "inocompat.h"

#include "RPLidarC1sim.h"
#include "RANSAC.h"
#include "StateMachine.h"
#include "AGVPins.h"
#include "Sensors.h"
#include <LSM6.h>

#include <vector>
#include <deque>

using namespace std;


RPLidarC1 lidar(&Serial1);
RPLidarHealth health;
RPLidarMeasurement m;

RANSAC ransacRight;
RANSAC ransacLeft;

sensors processIMU;

// Doesnt really do anything with the sim yet.
LSM6 imu;

StateMachine S;

//Double Ended Queues for both sides of the car that the LiDAR is reading into
//Using Deques due its ability to remove form the from and back, which is needed for our rolling buffer
deque<float> leftSideAngles,   leftSideDistances;
deque<float> rightSideAngles,  rightSideDistances;

//Thresh-hold for how far a wall should be before there needs to be gradual course correction 
float xsoftThreshhold = 127.0f;  // 127mm = 5 inches

//Consant distances for when only on wall is applicable in mm
const float desiredOneLineDistance = 203.20f;

//Limits the amount in a deque to around 3-5 seconds worth of LiDAR data.
// 18000 (angle and distance) -> ~5 seconds. 
// 9000 per side, with 4500 split between angle in distance.
static size_t dequeLimit = 150;


void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);
    delay(2000);

    pins p;
	
    pinMode(p.PWM1, OUTPUT);
    pinMode(p.PWM2, OUTPUT);
    pinMode(p.INA1, OUTPUT);
    pinMode(p.INA2, OUTPUT);
    pinMode(p.INB1, OUTPUT);
    pinMode(p.INB2, OUTPUT);

    Serial.println("Initializing RPLidar...");

    // Most of this just returns true
    if (!lidar.begin(460800, 4000)) {
        Serial.println("Failed to initialize RPLidar.");
        while (1) delay(1000);
    }

    if (lidar.get_health(&health)) {
        lidar.print_health(&health);
        if (health.status != RPLIDAR_STATUS_OK) {
            Serial.println("Warning: LiDAR not healthy, attempting reset...");
            lidar.reset();
            delay(2000);
        }
    } else {
        Serial.println("Failed to get health status.");
    }

    if (!lidar.start_scan()) {
        Serial.println("Failed to start scan");
        while (1) delay(1000);
    }
    Serial.println("Scan started successfully");

    
    Wire.begin();
    Wire.setClock(400000);
    imu.enableDefault();

    
    long sum = 0;
    int samples = 500;
    for (int i = 0; i < samples; i++) {
        imu.read();
        sum += imu.g.z;
        delay(2);
    }
    processIMU.biasZ = (float)sum / samples;

    Serial.println("Bias found: ");
    Serial.print(processIMU.biasZ);

    S.STATE = S.STOP;
}


void loop() {
    // Vectors to hold the converted LiDAR points

    static float lineDifference  = 0.0f;
    static float rightDistance   = 0.0f;
    static float leftDistance    = 0.0f;

    static float rightOnlyDistanceDifference = 0.0f;
    static float leftOnlyDistanceDifference  = 0.0f;

	//Timers for stop, searching_for_walls, and end_of_row states
    static unsigned long stopStartTime       = 0;
    static unsigned long timeSinceValidation = 0;
    static unsigned long timeSinceSearchShift = 0;

    static vector<points> rightCartesianConverted;
    static vector<points> leftCartesianConverted;

    // IMU doesnt work in sim right now
    imu.read();
    float rawZ     = imu.g.z;
    float centeredZ = rawZ - processIMU.biasZ;
    processIMU.filteredGyroZ = (processIMU.filteredGyroZ * (1.0f - processIMU.alpha))
                             + (centeredZ * processIMU.alpha);
    float zRateDegreesPerSecond = processIMU.filteredGyroZ * 0.070f;
    

    // Pulls measurements from datalayer
    if (lidar.get_measurement(&m)) {
		
		//Checks if detect angles are within a certain thresh-hold.
        //45 -> 135 degrees is right side
        //225 -> 315 is left side.
        if (m.angle > 45.0f && m.angle <= 135.0f) {
            rightSideAngles.push_back(m.angle);
            rightSideDistances.push_back(m.distance);
        } else if (m.angle > 225.0f && m.angle <= 315.0f) {
            leftSideAngles.push_back(m.angle);
            leftSideDistances.push_back(m.distance);
        }

		//If the size of one of the angle deques is greater than the dequeLimit (4500)...
		//Start a rolling buffer that will delete the beginning of the deque and add another data point.
        while (rightSideAngles.size() > dequeLimit) {
            rightSideAngles.pop_front();
            if (!rightSideDistances.empty()) rightSideDistances.pop_front();
        }        
        while (leftSideAngles.size() > dequeLimit) {
            leftSideAngles.pop_front();
            if (!leftSideDistances.empty()) leftSideDistances.pop_front();
        }

        // This will only start the RANSAC and STATEMACHINE process whennever a rotation starts
        if (m.start_flag) {
			// Wait until there is at-least 50 data points for both side
            if (leftSideAngles.size() > 50 || rightSideAngles.size() > 50) {
				// Clear vectors
                leftCartesianConverted.clear();
                rightCartesianConverted.clear();

                // Convert r and thada to x and y for RANSAC
                ransacRight.cartesianConversion(rightSideAngles, rightSideDistances, rightCartesianConverted);
                ransacLeft.cartesianConversion(leftSideAngles,  leftSideDistances,  leftCartesianConverted);

                // Try to find best-fit line through RANSACLoop
                ransacRight.RANSACLoop(rightCartesianConverted);
                ransacLeft.RANSACLoop(leftCartesianConverted);

                // Calculate distance from origin (the car) to the RANSAC lines
                ransacRight.distancetoLine();
                rightDistance = ransacRight.distance;

                ransacLeft.distancetoLine();
                leftDistance = ransacLeft.distance;

                // Calculation for finding the distance between RANSAC lines. Use this for PID as an error to correct.
                lineDifference = rightDistance - leftDistance;
				if (ArduinoCompat::g_dataLayer) {
                    ArduinoCompat::g_dataLayer->debug.rightLine = {
                        ransacRight.bestLine.a,
                        ransacRight.bestLine.b,
                        ransacRight.bestLine.c,
                        ransacRight.lineValidation()
                    };
                    ArduinoCompat::g_dataLayer->debug.leftLine = {
                        ransacLeft.bestLine.a,
                        ransacLeft.bestLine.b,
                        ransacLeft.bestLine.c,
                        ransacLeft.lineValidation()
					};
				}
			}
        }
	}        
    
    static unsigned long lastMotorUpdate = 0;
    if (millis() - lastMotorUpdate >= 10) { // Run at 100Hz
        lastMotorUpdate = millis();

        // Cache validation results so we don't call them multiple times per tick
        bool rightOk = ransacRight.lineValidation();
        bool leftOk  = ransacLeft.lineValidation();

        switch (S.STATE) {

            case S.STOP:
                S.stop();

                // Start the timer on the first frame we enter STOP
                if (stopStartTime == 0) stopStartTime = millis();

                // After 5 seconds, move off if at least one wall is visible
                if (millis() - stopStartTime > 5000) {
                    if (rightOk || leftOk) {
                        S.STATE = S.INBETWEEN_ROWS;
                        Serial.println("WALL FOUND: SWITCHING");
                    }
                }
                break;

            case S.INBETWEEN_ROWS:
                // Four possible wall visibility states — each has its own steering input

                // ============= Print Line difference here ==============
				if (ArduinoCompat::g_dataLayer) {
                    ArduinoCompat::g_dataLayer->debug.lineDifference = lineDifference;
                    ArduinoCompat::g_dataLayer->debug.zRate = zRateDegreesPerSecond;
					ArduinoCompat::g_dataLayer->debug.PIDResult = s.GetPIDResult();
				}
                if (rightOk && leftOk) {
                    // Both walls visible — use the signed difference to centre between them
                    timeSinceValidation = 0;
                    S.inbetween_rows(lineDifference, zRateDegreesPerSecond);

                } else if (rightOk && !leftOk) {
                    // Right wall only — maintain a fixed distance from it
                    rightOnlyDistanceDifference = desiredOneLineDistance - rightDistance;
                    S.inbetween_rows(rightOnlyDistanceDifference, zRateDegreesPerSecond);

                } else if (!rightOk && leftOk) {
                    // Left wall only — maintain a fixed distance from it
                    leftOnlyDistanceDifference = desiredOneLineDistance - leftDistance;
                    S.inbetween_rows(leftOnlyDistanceDifference, zRateDegreesPerSecond);

                } else {
                    // No walls visible — keep driving with last known values
                    // and start the timeout timer
                    S.inbetween_rows(lineDifference, zRateDegreesPerSecond);

                    if (timeSinceValidation == 0) timeSinceValidation = millis();

                    // If walls have been missing for 1.5 seconds, switch to searching
                    if (millis() - timeSinceValidation > 1500) {
                        S.STATE = S.SEARCHING_FOR_WALLS;
                        timeSinceSearchShift = timeSinceValidation;
                        timeSinceValidation  = 0;
                        Serial.println("No walls — searching...");
                    }
                }
                break;

            case S.SEARCHING_FOR_WALLS:
                // Gradually slow down while scanning for walls
                S.searching_for_walls();

                Serial.println("Searching...");

                // Return to normal driving as soon as either wall reappears
                if (rightOk || leftOk) {
                    S.STATE = S.INBETWEEN_ROWS;
                    Serial.println("Switching back to inbetween");
                }
                break;
        }
    }
}
