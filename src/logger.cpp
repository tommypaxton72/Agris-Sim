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
		 << "leftDistance,rightDistance,"
		 << "lineDifference,pidResult,zRate,state,"
		 << "leftValid,rightValid,"
         << "waypointX, waypointY\n";

	startTime = (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	
    std::cout << "[Logger] Opened: " << filename << "\n";
	
}

void Logger::LogData(const pose& p, const DataLayer& dataLayer) {
    // Package everything into a LogEntry and push onto the buffer.
    // No file I/O here — keeps World::Update() lightweight.
    LogEntry entry;
    entry.timestamp = (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() - startTime;
    entry.x = p.x;
    entry.y = p.y;
    entry.theta = p.theta;
    entry.leftPWM = dataLayer.leftMotor.PWM;
    entry.rightPWM = dataLayer.rightMotor.PWM;
    entry.leftDistance = dataLayer.debug.leftDistance;
    entry.rightDistance = dataLayer.debug.rightDistance;
    entry.lineDifference = dataLayer.debug.lineDifference;
    entry.pidResult = dataLayer.debug.PIDResult;
    entry.zRate = dataLayer.debug.zRate;
    entry.state = dataLayer.debug.state;
	entry.leftValid  = dataLayer.debug.leftValid;
	entry.rightValid = dataLayer.debug.rightValid;
    entry.waypoint.x = dataLayer.debug.waypoint.x;
    entry.waypoint.y = dataLayer.debug.waypoint.y;
    

    buffer.push_back(entry);

    // Flush to disk every flushInterval frames so we don't
    // accumulate too much in memory or lose data on a crash.
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
			 << e.leftDistance << "," << e.rightDistance << ","
			 << e.lineDifference << "," << e.pidResult << "," << e.zRate << "," << e.state << ","
			 << e.leftValid << "," << e.rightValid << "," << e.waypoint.x << "," << e.waypoint.y << "\n";
}

    buffer.clear();
}
