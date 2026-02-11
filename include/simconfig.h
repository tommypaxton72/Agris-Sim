#ifndef SIMCONFIG_H
#define SIMCONFIG_H

// Add to a new simconfig.h or just world.h
struct SimConfig {
    int windowWidth;
    int windowHeight;
    std::string windowTitle;
    int targetFPS;
    float dt;
    WorldSize worldSize;
};



#endif
