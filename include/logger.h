#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <vector>
#include <fstream>
#include "types.h"
#include "datalayer.h"


struct LogEntry {
    unsigned long timestamp;
    float x, y, theta;
    int leftPWM, rightPWM;
    float leftDistance, rightDistance;
    float lineDifference;
    float pidResult;
    float zRate;
    int state;
};

class Logger {
public:
    // Creates the file and writes the CSV header row.
    // Call once before the main loop starts.
    void OpenFile(const std::string& filename);

    // Packages pose and dataLayer into a LogEntry and pushes it onto the buffer.
    void LogData(const pose& p, const DataLayer& dataLayer);

    
    // Called automatically every flushInterval frames from Log(),
    // and manually from Close().
    void Flush();

    // Flushes any remaining buffered entries and closes the file.
    // Call when the simulation window closes.
    void Close();

private:
    std::ofstream file;
    std::vector<LogEntry> buffer;

    // How many frames to accumulate before writing to disk
    static constexpr int flushInterval = 60;
    int frameCount = 0;
    unsigned long startTime = 0;
};

#endif
