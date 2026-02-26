#ifndef AUTOCONTROL_H
#define AUTOCONTROL_H

#include "inocompat.h"
#include "RANSAC.h"
#include "StateMachine.h"
#include "datalayer.h"
#include <deque>
#include <vector>

class AutoControl {
public:
    AutoControl();

    void Setup();

    
    void Update(DataLayer& dataLayer);

    // Status accessors for Renderer / logging
    float RightDistance()  const { return rightDistance; }
    float LeftDistance()   const { return leftDistance; }
    float LineDifference() const { return lineDifference; }
    StateMachine::AGVSTATE State() const { return sm.STATE; }

private:
    RANSAC ransacRight;
    RANSAC ransacLeft;
    StateMachine sm;

    std::deque<float> leftAngles,  leftDistances;
    std::deque<float> rightAngles, rightDistances;
    static constexpr size_t DEQUE_LIMIT = 150;

    float rightDistance  = 0.0f;
    float leftDistance   = 0.0f;
    float lineDifference = 0.0f;

    unsigned long stopStartTime       = 0;
    unsigned long timeSinceValidation = 0;
    unsigned long lastMotorUpdate     = 0;

    static constexpr float ONE_LINE_DIST = 203.20f;

    void ProcessScan(const LidarData& data);
    void RunRANSAC();
    void RunStateMachine(DataLayer& dataLayer);
};

#endif
