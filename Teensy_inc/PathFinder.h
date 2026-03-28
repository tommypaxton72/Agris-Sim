#ifndef PATHFINDER_H
#define PATHFINDER_H

#ifndef SIM
#include <Arduino.h>
#else
#include "inocompat.h"
#endif

#include <cstdint>
#include "datastructs.h"
#include "Config.h"

// Which waypoint source to use as the lookahead target
// Local  — RANSAC generated, robot-relative, no transform needed
// Global — world frame position, requires pose transform
enum class WaypointMode {
    Local,   // use local waypoint from Perception directly
    Global   // use global waypoint, transform into robot frame first
};

class PathFinder {
    public:
        PathFinder();

        // Primary update — call every loop
        // Selects local or global waypoint based on availability
        // Stores results internally, read back with getters
        void UpdateControl(const Waypoint& localWaypoint,
                           const Waypoint& globalWaypoint,
                           const Pose& currentPose);

        // Getters
        const MotorCommands& GetMotorCommands() const { return motor; };

        // Steering angle in degrees — negative = left, positive = right
        // Derived from curvature, useful for logging and AutoForward compatibility
        float GetAngle() { return steeringAngle; };

        // Which waypoint source is currently active
        WaypointMode GetActiveMode() { return activeMode; };

    private:
        MotorCommands motor;

        float steeringAngle  = 0.0f; // degrees
        WaypointMode activeMode = WaypointMode::Local;

        // --- Waypoint selection ---
        // Prefers local waypoint when valid since it has no transform error
        // Falls back to global when local is not available
        // Returns the selected waypoint in robot-local frame (x = lateral offset)
        float SelectAndTransform(const Waypoint& local,
                                 const Waypoint& global,
                                 const Pose& pose);

        // --- Local waypoint ---
        // Local waypoint from Perception is already in robot frame
        // x = lateral offset from centerline, y = lookahead distance ahead
        // No transform needed — just return x directly
        float GetLocalLateralOffset(const Waypoint& local);

        // --- Global waypoint ---
        // Global waypoint is in world frame, must rotate into robot frame
        // Uses pose.theta (heading in radians) to align axes
        // Returns lateral offset x in robot frame
        float TransformGlobalToLocal(const Waypoint& global, const Pose& pose);

        // --- Pure pursuit core ---
        // Curvature = 2x / L^2
        // x = lateral offset, L = lookahead distance
        // Positive = turn right, negative = turn left
        float CalcCurvature(float localX);

        // --- Output calculation ---
        // Differential drive speed model:
        // left  = base * (1 - curvature * wheelbase/2)
        // right = base * (1 + curvature * wheelbase/2)
        // Also derives steeringAngle from curvature for logging
        void CalcOutputs(float curvature);
};

#endif