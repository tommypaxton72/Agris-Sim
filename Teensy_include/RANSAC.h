#ifndef RANSAC_h
#define RANSAC_h

#ifndef SIM
#include <Arduino.h>
#endif

#ifdef SIM
#include "inocompat.h"
#endif

#include <cstdint>
#include "datastructs.h"
#include "Config.h"

// This needs some looking at

struct rowPoints {
    float x[MAX_LIDAR_POINTS];
    float y[MAX_LIDAR_POINTS];
    uint16_t count = 0;
};

struct SeperatedPoints {
    rowPoints leftPoints;
    rowPoints rightPoints;
};


class RANSAC {
    public:
        RANSAC();
        Row RunRansac(const LidarData& lidar);
    private:
        RansacLine RunRANSACOnRow(const rowPoints& points);
        float CalcDistanceSq(const RansacLine& l, const Point& p);
        SeperatedPoints SeperatePoints(const LidarData& lidar);
        RansacLine FitLine(const Point& p1, const Point& p2);
        uint16_t CountInliers(const RansacLine& testLine, const rowPoints& row);
};


#endif
