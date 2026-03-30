#include "Pose.h"

// =============================================================================
// Tuning constants — change these to match your hardware
// =============================================================================

static const float TICKS_PER_REV        = 360.0f;
static const float WHEEL_DIAMETER_MM    = 100.0f;
static const float WHEEL_CIRCUMFERENCE  = WHEEL_DIAMETER_MM * 3.14159f;  // renamed
static const float TRACK_WIDTH          = 279.4f;   // renamed — was WHEELBASE_MM
static const float GYRO_SCALE           = (245.0f / 32768.0f) * (3.14159f / 180.0f);
static const float COMP_FILTER_ALPHA    = 0.98f;
static const float GPS_WEIGHT           = 0.3f;

// =============================================================================

Position::Position() {}

void Position::UpdatePose() {
    #ifdef SIM
    if (ArduinoCompat::g_simX)     currentPose.x     = *ArduinoCompat::g_simX;
    if (ArduinoCompat::g_simY)     currentPose.y     = *ArduinoCompat::g_simY;
    if (ArduinoCompat::g_simTheta) currentPose.theta = *ArduinoCompat::g_simTheta;
    return;
    #endif

    // Calculate time delta in seconds since last update
    uint32_t now = millis();
    float dt = (now - lastUpdateTime) / 1000.0f;
    lastUpdateTime = now;

    // Guard against first call or very large dt after a pause
    if (dt <= 0.0f || dt > 0.5f) return;

    // Read all sensors
    ReadIMU();
    // ReadCompass();
    // ReadEncoders();
    // ReadGPS();

    // Fuse sensor data into pose
    UpdateHeading(dt);
    UpdatePosition(dt);
}

void Position::ReadIMU() {
    #ifndef SIM
    imu.read(); // populates imu.g.x/y/z and imu.a.x/y/z
    #endif
}

void Position::ReadCompass() {
    #ifndef SIM
    compass.read(); // populates compass heading
    #endif
}

void Position::ReadEncoders() {
    // Store previous tick counts before reading new ones
    lastLeftTicks  = leftTicks;
    lastRightTicks = rightTicks;

    #ifndef SIM
    // Replace these with your actual encoder read calls
    // e.g. leftTicks = encoder_left.read();
    //      rightTicks = encoder_right.read();
    #else
    // Sim: ticks stay at 0 — override from simulator if needed
    #endif
}

void Position::ReadGPS() {
    #ifndef SIM
    // Replace with your actual GPS library read
    // Set gpsNewFix = true and update gpsX/gpsY when a new fix arrives
    // e.g.:
    // if (gps.newData()) {
    //     gpsX = gps.getX();
    //     gpsY = gps.getY();
    //     gpsNewFix = true;
    // }
    #endif
}

void Position::UpdateHeading(float dt) {
    // --- Gyro contribution ---
    // Integrate gyro Z angular rate to get heading change this tick
    #ifndef SIM
    float gyroRate = imu.g.z * GYRO_SCALE;  // raw to rad/s
    #else
    float gyroRate = 0.0f;
    #endif
    float gyroHeading = filteredHeading + gyroRate * dt;

    // --- Compass contribution ---
    // Convert compass reading to radians
    // QMC5883L gives degrees 0-360, remap to match robot forward = 0
    #ifndef SIM
    float compassDeg     = compass.getAzimuth();
    float compassHeading = compassDeg * (3.14159f / 180.0f);
    #else
    float compassHeading = 0.0f;
    #endif

    // --- Complementary filter ---
    // Blends gyro (short term accurate) with compass (long term accurate)
    // Alpha close to 1.0 = heavily trust gyro, small compass correction each tick
    filteredHeading = COMP_FILTER_ALPHA * gyroHeading
                    + (1.0f - COMP_FILTER_ALPHA) * compassHeading;

    currentPose.theta = filteredHeading;
}

void Position::UpdatePosition(float dt) {
    // --- Encoder odometry ---
    // How many ticks each wheel moved this loop
    int32_t dLeft  = leftTicks  - lastLeftTicks;
    int32_t dRight = rightTicks - lastRightTicks;

    // Convert tick delta to mm traveled per wheel
    float leftMM  = TicksToMM(dLeft);
    float rightMM = TicksToMM(dRight);

    // Average forward distance and heading change from differential drive
    float distanceMM  = (leftMM + rightMM) / 2.0f;
    float dTheta      = (rightMM - leftMM) / TRACK_WIDTH; // radians

    // Dead reckoning — project forward distance along current heading
    // Use midpoint heading for better arc approximation
    float midTheta = currentPose.theta + dTheta / 2.0f;
    currentPose.x += distanceMM * cosf(midTheta);
    currentPose.y += distanceMM * sinf(midTheta);

    // Update velocity in mm/s for pose consumers
    currentPose.velocity = distanceMM / dt;

    // --- GPS correction ---
    // When a new fix arrives, blend GPS position into odometry estimate
    // This prevents odometry drift accumulating over long runs
    if (gpsNewFix) {
        currentPose.x = (1.0f - GPS_WEIGHT) * currentPose.x + GPS_WEIGHT * gpsX;
        currentPose.y = (1.0f - GPS_WEIGHT) * currentPose.y + GPS_WEIGHT * gpsY;
        gpsNewFix = false;
    }
}

float Position::TicksToMM(int32_t ticks) {
    return ((float)ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
}

void Position::SetEORPose() {
    poseEOR =  currentPose;
}

Waypoint Position::GetEORWaypoint() {
    Waypoint EORWaypoint;
    EORWaypoint.x = EOR_LOOKAHEAD * sin(currentPose.theta);
    EORWaypoint.y = EOR_LOOKAHEAD * cos(currentPose.theta);
    return EORWaypoint;
}

