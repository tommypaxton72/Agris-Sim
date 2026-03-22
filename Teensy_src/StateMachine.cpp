#include "StateMachine.h"

//Notes
// Ransac linefitting for row detection?
// IMU/GPS Fusion for better localization
// Kalman Filter for IMU/GPS Fusion
// Deskewing Lidar Data
// Do we want to constantly check state?


StateMachine::StateMachine() {
    mState = ManualDrive;
    sState = START;
}

void StateMachine::Start() {
    mState = ManualDrive;
    sState = START;
    // (Do Once)
        // Reset Variables
        // GetLocalCoordPose
        // Get pointer to ScanData


}

void StateMachine::MainTransitionTo(MainState newState) {
    if (newState == AutoDrive && mState == ManualDrive) {
        // Reset Variables and States for Manual Control
        
        
        mState = AutoDrive;

    } else if (newState == ManualDrive && mState == AutoDrive) {
        // Reset Variables and States for Auto Control
        
        mState = ManualDrive;
    }
}

// Main State-Machine Switch
void StateMachine::Run() {
    #ifndef SIM
    // Check for manual/autodrive serial
    #else
    mState = AutoDrive; 
    #endif

    switch (mState) {
        case AutoDrive:

            RunAuto();
            break;

        case ManualDrive:
            RunManual();
            break;

        default:
            break;

    }
}

#ifndef SIM
// Manual Control Code
void StateMachine::RunManual() {
    // Manual Motor Control
    if (agvrx.Output.LWForward > 0 && agvrx.Output.LWReverse == 0) {
      analogWrite(LPWM,agvrx.Output.LWForward);
      digitalWrite(LINA, HIGH);
      digitalWrite(LINB, LOW);
    } else if (agvrx.Output.LWReverse > 0 && agvrx.Output.LWForward == 0) {
      analogWrite(LPWM,agvrx.Output.LWReverse);
      digitalWrite(LINB, HIGH);
      digitalWrite(LINA, LOW);
    } else {
      digitalWrite(LINA, LOW);
      digitalWrite(LINB, LOW);
      digitalWrite(LPWM, LOW);
    }
    if (agvrx.Output.RWForward > 0 && agvrx.Output.RWReverse == 0) {
      analogWrite(RPWM, agvrx.Output.RWForward);
      digitalWrite(RINA, HIGH);
      digitalWrite(RINB, LOW);
    } else if (agvrx.Output.RWReverse > 0 && agvrx.Output.RWForward == 0) {
      analogWrite(RPWM,agvrx.Output.RWReverse);
      digitalWrite(RINB, HIGH);
      digitalWrite(RINA, LOW);
    } else {
      digitalWrite(RINA, LOW);
      digitalWrite(RINB, LOW);
      digitalWrite(RPWM, LOW);
    }
    // Manual Solenoid Control
    digitalWrite(RInnerSolenoid, agvrx.Output.RISolenoid ? HIGH : LOW);
    digitalWrite(LInnerSolenoid, agvrx.Output.LISolenoid ? HIGH : LOW);
    digitalWrite(ROuterSolenoid, agvrx.Output.ROSolenoid ? HIGH : LOW);
    digitalWrite(LOuterSolenoid, agvrx.Output.LOSolenoid ? HIGH : LOW);
    digitalWrite(ROuterSolenoid, agvrx.Output.ROSolenoid ? HIGH : LOW);
    digitalWrite(LOuterSolenoid, agvrx.Output.LOSolenoid ? HIGH : LOW);
    digitalWrite(OutPump, agvrx.Output.Pump ? HIGH : LOW);

    #ifdef PrintDebug
    Serial.printf("LWForward: %d, LWReverse: %d, RWForward: %d, RWReverse: %d\n",
      agvrx.Output.LWForward,
      agvrx.Output.LWReverse,
      agvrx.Output.RWForward,
      agvrx.Output.RWReverse);
      delay(250);
    #endif


}
#else
void StateMachine::RunManual() {}
#endif

// AutoDrive Control Code
void StateMachine::RunAuto() {

    // Have state detection run alittle slower
    if (millis() - lastDetectionTime > STATEDETECTION_DELAY) {
        StateMachine::StateDetection();
        lastDetectionTime = millis();
    }
    
    if (perception.CheckCollision()) {
        sState = COLLISION_AVOIDANCE;
    }
    
    switch (sState) {
        case START:
            // Need to add an actual start condition
            sState = INBETWEEN_ROWS;
            break;
        case INBETWEEN_ROWS:
            InbetweenRows();
            break;
        
        case END_OF_ROW:
            EndofRow();
            break;

        case COLLISION_AVOIDANCE:
            CollisionAvoidance();
            break;

        default:
            // Defaults when no row detected on either side
            // Might cause an issue with the end of row state moving so far out

            break;
    }
}

void StateMachine::InbetweenRows() {

    perception.UpdateFilteredLidar();
    
    perception.UpdateRANSAC();

    perception.GenerateWaypoint();

    robotPose.UpdatePose();

    GenerateGlobalWaypoint();

    control.UpdateControl(
        perception.GetLocalWaypoint(),  // from RANSAC — preferred
        globalWaypoints[waypointIndex], // fallback when RANSAC loses lines
        robotPose.GetCurrentPose());
    
    motorControl.AutoForward(control.GetLeftPWM(), control.GetRightPWM());

    // Somewhere in here is where you recieve spray commands from Raspberry Pi

    #ifdef SIM
    if (ArduinoCompat::g_dataLayer) {
        DataLayer& dl = *ArduinoCompat::g_dataLayer;
        dl.debug.leftValid    = perception.LeftLineValid();
        dl.debug.rightValid   = perception.RightLineValid();
        dl.debug.leftDistance = perception.GetLeftRansac().b;   // x intercept of left line
        dl.debug.rightDistance= perception.GetRightRansac().b;  // x intercept of right line
        dl.debug.lineDifference = perception.GetLocalWaypoint().x; // lateral offset
        dl.debug.PIDResult    = control.GetAngle();
        dl.debug.state        = (int)sState;
    }
    #endif

}

// This needs some work
void StateMachine::GenerateGlobalWaypoint() {
    
    if (waypoint_OldTimer - millis() > WAYPOINT_TIMER) {
        waypoint_OldTimer = millis();
        if (waypointIndex <= MAX_WAYPOINTS) {
            waypointIndex++;
        } else {
            waypointIndex = 0;
        }
    }
    
    Waypoint localWaypoint = perception.GetLocalWaypoint();
    // Pose pose = robotPose.GetGlobalPose();
    // waypoint[waypointIndex] = localWaypoint + pose;
}
    // Reset Variables and Stat

void StateMachine::EndofRow() {
    // Pointer to ScanData
    // IMU/GPS Fusion
    // Find End of Row
    // Pass End of Row by z amount
    // Detect which side row is on (use leftRow and rightRow?)
    // Turn towards next row using constant radius turn with row edge as reference
    motorControl.Stop();
}

void StateMachine::Turning() {

}

void StateMachine:: Alignment() {

}

void StateMachine::CollisionAvoidance() {
    // Pointer to ScanData
    // Find clear path to reverse to previous waypoint?
    // Maybe follow previous waypoints back 3
    // Stop
    // Move to inbetween rows state once clear
    motorControl.Stop();

}

void StateMachine::StateDetection() {
    // Pointer to ScanData
    // IMU/GPS Fusion
    // Map Building
    // Sanity Checks
    // State Transition Logic
        //Start
            //
        //Inbetween Rows
            //
        //End of Row
            //

    switch (sState) {

        case INBETWEEN_ROWS:
            if (!perception.LeftLineValid() && !perception.RightLineValid()) {
                noLineCounter++;
            } else {
                noLineCounter = 0;
            }
            if (noLineCounter >= 5) {
                TransitionTo(END_OF_ROW);
            }
            break;
        default:
            break;
        }
}

void StateMachine::TransitionTo(SubState newState) {
    
    sState = newState;

}

bool StateMachine::InbetweenRowCondition() {
    // Logic to determine if inbetween rows
    return true;
    // Need to figure out a way to test multiple cycles before determining

}

bool StateMachine::EndOfRowCondition() {
    // Logic to determine if at end of row
    return false;
}


void StateMachine::ResetAll() {
    // Reset all variables and states
}

void StateMachine::ResetAuto() {
    // Reset variables and states after auto drive
}

void StateMachine::ResetManual() {
    // Reset variables and states after manual intervention
}


