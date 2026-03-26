#ifndef STATEMACHINE_H
#define STATEMACHINE_H


#ifndef SIM
#include <Arduino.h>
#include "AGRIS.h"
#endif

#ifdef SIM
#include "inocompat.h"
#endif

#include <cstdint>

#include "datastructs.h"
#include "Config.h"

#include "Perception.h"
#include "Pose.h"
#include "MotorControl.h"
#include "PathFinder.h"


enum MainState {
    AutoDrive,
    ManualDrive
};

// Do we need enum class or enum?
enum SubState {
    START,
    INBETWEEN_ROWS,
    END_OF_ROW,
    TURNING,
    ALIGNING,
    ESTOP,
    COLLISION_AVOIDANCE
};



class StateMachine {
    public:
        StateMachine();

        void Start();
        void Run();

        // Getters
        const Waypoint& GetGlobalWaypoint(int idx) const { return globalWaypoints[idx]; }
    private:
        // Classes
        Perception perception;
        Position robotPose;
        MotorController motorControl;
        PathFinder control;

        // Enums
        MainState mState = ManualDrive;
        SubState sState = START;


        // Structs
        Waypoint globalWaypoints[MAX_WAYPOINTS];

        uint32_t lastDetectionTime = 0;
        uint8_t noLineCounter = 0;
        uint8_t waypointIndex = 0;
        uint32_t waypoint_OldTimer = 0;

        void MainTransitionTo(MainState newState);
        void RunAuto();
        void RunManual();
        
        void StateDetection();

        void TransitionTo(SubState newState);
        void InbetweenRows();
        void EndofRow();
        void CollisionAvoidance();
        void Turning();
        void Alignment();

        void GenerateGlobalWaypoint();

        bool InbetweenRowCondition();
        bool EndOfRowCondition();

        void ResetAll();
        void ResetAuto();
        void ResetManual();
        
};


#endif