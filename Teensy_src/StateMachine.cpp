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
// Would like to be able to set first End of Row turn direction with manual control
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
// Need to have empty definition for Sim
void StateMachine::RunManual() {}
#endif


// AutoDrive Control Code
void StateMachine::RunAuto() {

    
    perception.UpdateFilteredLidar();

    // Check collision
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

        case ALIGNING:
            Aligning();
            break;

        case TURNING:
            Turning();
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


// This needs some work
void StateMachine::GenerateGlobalWaypoint() {
    
    if (millis() - waypoint_OldTimer > WAYPOINT_TIMER) {
        waypoint_OldTimer = millis();
        if (waypointIndex < MAX_WAYPOINTS - 1) {
            waypointIndex++;
        } else {
            waypointIndex = 0;
        }


        Waypoint localWaypoint = perception.GetLocalWaypoint();
        Pose pose = robotPose.GetCurrentPose();

        // CW RANSAC transform: +x=rightward, forward=-y, localWaypoint.y stored as positive forward magnitude
        float wx =  sinf(pose.theta) * localWaypoint.x + cosf(pose.theta) * localWaypoint.y;
        float wy = -cosf(pose.theta) * localWaypoint.x + sinf(pose.theta) * localWaypoint.y;

        globalWaypoints[waypointIndex].x = wx + pose.x;
        globalWaypoints[waypointIndex].y = wy + pose.y;
        globalWaypoints[waypointIndex].valid = true;


        // Global waypoitns dont work quite right with the sim because even though its global waypoint its not set on the reference
        // Frame of the world.
        #ifdef SIM
        std::cout << "[WP " << waypointIndex << "] x=" << globalWaypoints[waypointIndex].x
                  << " y=" << globalWaypoints[waypointIndex].y
                  << " localValid=" << localWaypoint.valid << "\n";
        #endif
    }
}

// State Functions
void StateMachine::InbetweenRows() {

    perception.UpdateRANSAC();

    perception.GenerateWaypoint();

    robotPose.UpdatePose();

    GenerateGlobalWaypoint();

    control.UpdateControl(perception.GetLocalWaypoint(), globalWaypoints[waypointIndex], robotPose.GetCurrentPose());
    
    motorControl.AutoForward(control.GetMotorCommands().leftMotor.PWM, control.GetMotorCommands().rightMotor.PWM);

    // Somewhere in here is where you recieve spray commands from Raspberry Pi

    if (!EndOfRowCondition) {
        TransitionTo(END_OF_ROW);
    }

}




void StateMachine::EndofRow() {

    // Pass End of Row by z amount
    // Detect which side row is on (use leftRow and rightRow?)
    // Turn towards next row using constant radius turn with row edge as reference
    
    // Could also take lidar data and search behind for closest points and create ransac line 
    //from that to get end of row line?
    perception.UpdateFilteredLidar();
    perception.SetLidarEndOfRow(10);
    perception.SetEndOfRowLine();
    RansacLine lineEOR = perception.GetEndOfRowLine();

    #ifdef DEBUG
    Serial.print("[EOR RANSAC] valid=");
    Serial.print(lineEOR.valid);
    Serial.print(" m=");
    Serial.print(lineEOR.m);
    Serial.print(" b=");
    Serial.println(lineEOR.b);
    #endif
    // Could also get the lidar data from the closest points and determine distance 
    // using angle and distance to get the rectangular distance from closest corn plants
    
    // I also like the idea of using old stored ransac lines to create last few waypoints?
    // With the robot being so far out of the row im worried it might pick up other rows.
    
    // Get Distance of line from robot
    // Line y = mx + b, b is where the 
    Pose current = robotPose.GetCurrentPose();
    float dx = current.x - EORPose.x;
    float dy = current.y - EORPose.y;
    
    
    if (TurningCondition()) {
        TransitionTo(TURNING);
    }

}


void StateMachine::Turning() {
    robotPose.UpdatePose();
    Pose pose = robotPose.GetCurrentPose();


    if (!turnInitialized) {
        startHeading = pose.theta;

        TurnDirection dir = (turnRowCount % 2 == 0) ? TurnDirection::Right : TurnDirection::Left;
        float sign = (dir == TurnDirection::Left) ? 1.0f : -1.0f;
        targetHeading = startHeading + sign * 3.14159f;
        control.SetTurnStart(startHeading);
        turnInitialized = true;
    }

    TurnDirection dir = (turnRowCount % 2 == 0) ? TurnDirection::Right : TurnDirection::Left;
    float rowDist = perception.GetAvgRowDistance();

    control.UpdateTurn(rowDist, dir, pose);

    motorControl.AutoForward(
        control.GetMotorCommands().leftMotor.PWM,
        control.GetMotorCommands().rightMotor.PWM
    );
    float test = control.GetHeadingProgress();
    
    if (AligningCondition()) {
        TransitionTo(ALIGNING);
    }
}

void StateMachine:: Aligning() {
    motorControl.Stop();
}

void StateMachine::CollisionAvoidance() {
    // Find clear path to reverse to previous waypoint?
    // Maybe follow previous waypoints back 3
    // Stop
    // Move to inbetween rows state once clear
    motorControl.Stop();

}

// State transition function
void StateMachine::TransitionTo(SubState newState) {
    if (newState == TURNING) {
        turnInitialized = false;  // arm the one-time heading snapshot
    }
    if (sState == TURNING && newState != TURNING) {
        turnRowCount++;            // alternate direction on next turn
    }
    sState = newState;
}

// State Transition Logic
bool StateMachine::InbetweenRowCondition() {
    // Logic to determine if inbetween rows
    return true;

}

bool StateMachine::EndOfRowCondition() {
    // Logic to determine if at end of row
    if (sState == INBETWEEN_ROWS) {
        uint8_t rowState = perception.CheckRows();
        
        if (rowState == 1 || rowState == 2 || rowState == 3) {
            noLineCounter++;
        } else {
            noLineCounter = 0;
        }
        if (noLineCounter >= 5) {
            // Clean this up later and put it in its respective files.
            EORWaypoint = robotPose.GetEORWaypoint();
            EORPose = robotPose.GetCurrentPose();

            return true;
            
        } else {
            return false;
        }
    }
    return false;
}

bool StateMachine::TurningCondition() {
    if (sState == END_OF_ROW) {
        Pose current = robotPose.GetCurrentPose();
        float dx = current.x - EORPose.x;
        float dy = current.y - EORPose.y;
        float dist = sqrtf(dx*dx + dy*dy);
        return dist >= EOR_LOOKAHEAD;
    }
    return false;
}

bool StateMachine::AligningCondition() {
    // If compass heading 
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


#ifdef SIM
Debug StateMachine::GetDebug() {
    Debug debug;
    
    // Motor commands
    debug.motor = ArduinoCompat::g_dataLayer->motor;
    debug.RansacLines = perception.GetRowData();
    debug.lineEOR = perception.GetEndOfRowLine();
    debug.assumedPose = robotPose.GetCurrentPose();
    debug.lWaypoint = perception.GetLocalWaypoint();
    debug.currentWaypointIndex = GetWaypointIndex();
    for (int i = 0; i < MAX_WAYPOINTS; i++) {
        debug.gWaypoint[i] = GetGlobalWaypoint(i);
    }
    debug.state = (int)sState;

    return debug;
}
#endif