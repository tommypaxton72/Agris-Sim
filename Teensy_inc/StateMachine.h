#ifndef STATEMACHINE_H
#define STATEMACHINE_H


#ifndef SIM
#include <Arduino.h>
#include "AGRIS.h"
#endif

#ifdef SIM
#include "inocompat.h"
#include <iostream>
#endif

#include <cstdint>


#include "datastructs.h"
#include "Config.h"

#include "Perception.h"
#include "Pose.h"
#include "MotorControl.h"
#include "PathFinder.h"

/*
================================ State Machine ==================================

*/






class StateMachine {
    public:
        StateMachine();

        void Start();
        void Run();

        // Getters
        const SubState GetDriveState() const { return sState; };

        const Waypoint& GetGlobalWaypoint(int idx) const { return globalWaypoints[idx]; };
        const uint8_t& GetWaypointIndex() const { return waypointIndex; };

        const Row& GetRansacRows() const { return perception.GetRowData(); };
        const LidarData& GetLidarData() const { return perception.GetLidarData(); };

        const Pose& GetCurrentPose() const { return robotPose.GetCurrentPose(); };
        
        const MotorCommands& GetMotorCommands() const { return control.GetMotorCommands(); };

        // Setters
        

        #ifdef SIM
        Debug GetDebug();
        #endif
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

        // Testing idea of keeping waypoint and other data for end of row.
        Pose EORPose;
        Waypoint EORWaypoint;

        bool    turnInitialized = false;
        float   startHeading    = 0.0f;
        float   targetHeading   = 0.0f;
        int     turnRowCount    = 0;

        void MainTransitionTo(MainState newState);
        void RunAuto();
        void RunManual();
        
        void StateDetection();

        void TransitionTo(SubState newState);
        void InbetweenRows();
        void EndofRow();
        void CollisionAvoidance();
        void Turning();
        void Aligning();

        void GenerateGlobalWaypoint();

        bool InbetweenRowCondition();
        bool EndOfRowCondition();
        bool TurningCondition();
        bool AligningCondition();



        void ResetAll();
        void ResetAuto();
        void ResetManual();

};


#endif