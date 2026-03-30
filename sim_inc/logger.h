#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <vector>
#include <fstream>
#include "types.h"
#include "datastructs.h"

struct LogEntry {
    unsigned long timestamp;
    float x, y, theta;
    int leftPWM, rightPWM;
    float leftDistance, rightDistance;
    float lineDifference;
    int state;
    bool leftValid  = false;
    bool rightValid = false;
    Row row;
    RansacLine EORLine;
    Waypoint lWaypoint;
    Waypoint gWaypoint;
};

class Logger {
public:
    void OpenFile(const std::string& filename);
    void LogData(const pose& p, const Debug& debug);
    void Flush();
    void Close();

private:
    std::ofstream file;
    std::vector<LogEntry> buffer;
    static constexpr int flushInterval = 60;
    int frameCount = 0;
    unsigned long startTime = 0;
};

#endif
