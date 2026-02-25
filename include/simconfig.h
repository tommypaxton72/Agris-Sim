#ifndef SIMCONFIG_H
#define SIMCONFIG_H

#include <string>
#include "types.h"

struct SimConfig {
    int windowWidth;
    int windowHeight;
    std::string windowTitle;
    int targetFPS;
    float dt;
    WorldSize worldSize;
};



#endif
