#include "PathFinder.h"

// =============================================================================
// Tuning constants
// =============================================================================

// Base forward speed — both motors start here before steering correction
static const float BASE_SPEED = (float)CRUISE_SPEED;

// Half the wheelbase — used in differential drive speed model
static const float HALF_WHEELBASE = WHEELBASE_MM / 2.0f;

// Lookahead distance in mm — how far ahead the robot aims
// Larger = smoother but slower to correct
// Smaller = more reactive but can oscillate
static const float LOOKAHEAD = (float)LOOKAHEAD_DISTANCE;

// Maximum steering angle in degrees — clamps output so robot doesnt spin in place
static const float MAX_STEERING_DEG = 45.0f;

// =============================================================================

PathFinder::PathFinder() {}

void PathFinder::UpdateControl(const Waypoint& localWaypoint,
                               const Waypoint& globalWaypoint,
                               const Pose& currentPose) {

    // Step 1 — select best available waypoint and get lateral offset in robot frame
    float lateralOffset = SelectAndTransform(localWaypoint, globalWaypoint, currentPose);

    // Step 2 — pure pursuit curvature from lateral offset
    float curvature = CalcCurvature(lateralOffset);

    // Step 3 — convert curvature to PWM and steering angle
    CalcOutputs(curvature);
}

float PathFinder::SelectAndTransform(const Waypoint& local,
                                     const Waypoint& global,
                                     const Pose& pose) {

    // Prefer local waypoint — it comes directly from RANSAC in robot frame
    // and has no accumulated transform error from pose estimation
    if (local.valid) {
        activeMode = WaypointMode::Local;
        return GetLocalLateralOffset(local);
    }

    // Fall back to global waypoint when local is unavailable
    // e.g. at end of row when RANSAC has lost both lines
    if (global.valid) {
        activeMode = WaypointMode::Global;
        return TransformGlobalToLocal(global, pose);
    }

    // Neither waypoint is valid — return zero offset, robot drives straight
    // StateMachine should handle this case by stopping or changing state
    activeMode = WaypointMode::Local;
    return 0.0f;
}

float PathFinder::GetLocalLateralOffset(const Waypoint& local) {
    // Local waypoint is already in robot frame from Perception::GenerateWaypoint()
    // x = lateral distance from centerline
    // negative = waypoint is left of robot center
    // positive = waypoint is right of robot center
    return local.x;
}

float PathFinder::TransformGlobalToLocal(const Waypoint& global, const Pose& pose) {
    // Translate: vector from robot to waypoint in world frame
    float dx = global.x - pose.x;
    float dy = global.y - pose.y;

    // Rotate by -theta to align with robot forward axis
    // pose.theta = heading in radians, 0 = forward, positive = counterclockwise
    float localX =  dx * cosf(-pose.theta) - dy * sinf(-pose.theta);
    // localY is forward distance — not needed for pure pursuit lateral correction
    // float localY = dx * sinf(-pose.theta) + dy * cosf(-pose.theta);

    return localX;
}

float PathFinder::CalcCurvature(float localX) {
    // Pure pursuit formula: curvature = 2x / L^2
    // Derivation: fit a circle through robot origin and lookahead point
    // Positive curvature = arc curves right
    // Negative curvature = arc curves left
    return (2.0f * localX) / (LOOKAHEAD * LOOKAHEAD);
}

void PathFinder::CalcOutputs(float curvature) {
    // --- Steering angle ---
    // Convert curvature (1/radius) to degrees for logging and AutoForward compat
    // angle = atan(curvature * wheelbase) converted to degrees
    // clamped to MAX_STEERING_DEG so value stays meaningful
    float angleRad  = atanf(curvature * WHEELBASE_MM);
    float angleDeg  = angleRad * (180.0f / 3.14159f);
    steeringAngle   = constrain(angleDeg, -MAX_STEERING_DEG, MAX_STEERING_DEG);

    // --- Differential drive PWM ---
    // Speed difference between wheels creates the arc curvature
    // left  = base * (1 - curvature * halfWheelbase)
    // right = base * (1 + curvature * halfWheelbase)
    //
    // Positive curvature (turn right):
    //   left speeds up, right slows down -> robot arcs right
    // Negative curvature (turn left):
    //   left slows down, right speeds up -> robot arcs left
    float left  = BASE_SPEED * (1.0f - curvature * HALF_WHEELBASE);
    float right = BASE_SPEED * (1.0f + curvature * HALF_WHEELBASE);

    // Clamp to valid PWM range
    motor.leftMotor.PWM  = (uint8_t)constrain(left,  0.0f, 255.0f);
    motor.rightMotor.PWM = (uint8_t)constrain(right, 0.0f, 255.0f);
}