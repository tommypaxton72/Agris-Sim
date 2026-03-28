#include "sketch.h"
#include "StateMachine.h"

StateMachine sm;

void setup() {
    sm.Start();
}

void loop() {

    sm.Run();

    #ifdef SIM
    if (ArduinoCompat::g_dataLayer)
        *ArduinoCompat::g_dataLayer = sm.GetDebug();
    #endif
}