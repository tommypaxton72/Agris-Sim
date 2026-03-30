#include "logger.h"
#include <iostream>
#include <chrono>

void Logger::OpenFile(const std::string& filename) {
    file.open(filename);
    if (!file.is_open()) {
        std::cerr << "[Logger] Failed to open file: " << filename << "\n";
        return;
    }

    file << "timestamp,x,y,theta,"
         << "leftPWM,rightPWM,"
         << "leftSlope,rightSlope,"
         << "leftDistance,rightDistance,"
         << "lineDifference,state,"
         << "leftValid,rightValid,"
         << "EORSlope,EORIntercept,"
         << "lWaypointX,lWaypointY,"
         << "gWaypointX,gWaypointY\n";

    startTime = (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    std::cout << "[Logger] Opened: " << filename << "\n";
}

void Logger::LogData(const pose& p, const Debug& debug) {
    LogEntry entry;
    entry.timestamp = (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() - startTime;
    entry.x     = p.x;
    entry.y     = p.y;
    entry.theta = p.theta;

    entry.leftPWM  = debug.motor.leftMotor.PWM;
    entry.rightPWM = debug.motor.rightMotor.PWM;

    entry.leftDistance  = debug.RansacLines.leftLine.b;
    entry.rightDistance = debug.RansacLines.rightLine.b;
    entry.lineDifference = debug.RansacLines.leftLine.b + debug.RansacLines.rightLine.b;
    entry.row = debug.RansacLines;
    entry.EORLine = debug.lineEOR;
    entry.state = debug.state;

    entry.leftValid  = debug.RansacLines.leftLine.valid;
    entry.rightValid = debug.RansacLines.rightLine.valid;
    
    entry.lWaypoint = debug.lWaypoint;
    entry.gWaypoint = debug.gWaypoint[debug.currentWaypointIndex];

    buffer.push_back(entry);

    frameCount++;
    if (frameCount >= flushInterval) {
        Flush();
        frameCount = 0;
    }
}

void Logger::Close() {
    Flush();
    if (file.is_open()) {
        file.close();
        std::cout << "[Logger] Closed.\n";
    }
}

void Logger::Flush() {
    if (!file.is_open()) return;
    for (const auto& e : buffer) {
        file << e.timestamp << "," << e.x << "," << e.y << "," << e.theta << ","
             << e.leftPWM << "," << e.rightPWM << ","
             << e.row.leftLine.m << "," << e.row.rightLine.m << ","
             << e.leftDistance << "," << e.rightDistance << ","
             << e.lineDifference << "," << e.state << ","
             << e.leftValid << "," << e.rightValid << ","
             << e.EORLine.m << "," << e.EORLine.b << ","
             << e.lWaypoint.x << "," << e.lWaypoint.y <<  ","
             << e.gWaypoint.x << "," << e.gWaypoint.y << "\n";
    }
    buffer.clear();
}
