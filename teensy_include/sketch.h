#ifndef SKETCH_H
#define SKETCH_H

// =============================================================================
// sketch.h
//
// Forward declares the two entry points that match a Teensy/Arduino sketch.
// Robot includes this so it can call setup() once during LoadConfig()
// and loop() every frame inside UpdateControl().
//
// The actual implementations live in sketch.cpp — that file is the
// Teensy code and should never need to include robot.h or world.h.
// The dependency only goes one way: Robot knows about the sketch,
// the sketch does not know about Robot.
// =============================================================================

void setup();
void loop();

#endif
