#include "sketch.h"
#include "StateMachine.h"


StateMachine sm;

void setup() {
    sm.Start();
}

void loop() {

    sm.Run();
    
}