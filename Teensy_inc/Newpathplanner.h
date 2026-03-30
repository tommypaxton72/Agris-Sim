#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#ifndef SIM
#include <Arduino.h>
#else
#include "inocompat.h"
#endif

#include "datastructs.h"
#include "Config.h"

/*
=========================== Path Planner =============================
Implementation for a pure pursuit path planner for a differential drive system
Takes in Pose of robot and a waypoint and calculates the commanded heading at the time of call.

For End of Row, Calculates heading to maintain course torwards final wwaypoint.

For Turning calculations, Takes in current pose and estimated row width to calculate a constant radius turn.
*/

class PathPlanner {
    public:
        PathPlanner();
        void Update(SubState state, );
    private:
        void 

}


#endif