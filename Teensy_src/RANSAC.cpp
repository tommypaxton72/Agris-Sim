#include "RANSAC.h"

// RANSAC implementation
RANSAC::RANSAC() {}

Row RANSAC::RunRansac(const LidarData& lidar) {
    Row row;

    SeperatedPoints rows = SeperatePoints(lidar);
    
    row.leftLine = RunRANSACOnRow(rows.leftPoints);
    row.rightLine = RunRANSACOnRow(rows.rightPoints);
    
    // At some point we should maybe validate the lines to make sure they are reasonable
    // ValidateScan(row.leftLine);
    // ValidateScan(row.rightLine);

    return row;
    

}

// RANSAC on a single row of points
RansacLine RANSAC::RunRANSACOnRow(const rowPoints& points) {
    if (points.count < 2) {
        return RansacLine{0, 0, 0, false};
    }
    RansacLine bestLine = {0.0f, 0.0f, 0, false};
    uint16_t bestInliers = 0;

    for (uint16_t iter = 0; iter < RANSAC_MAX_ITERATIONS; iter++) {
        uint16_t idx1 = rand() % points.count;
        uint16_t idx2 = rand() % points.count;
        if (idx1 == idx2) {
            continue;
        }
        Point p1 = {points.x[idx1], points.y[idx1]};
        Point p2 = {points.x[idx2], points.y[idx2]};
        if (fabs(p1.y - p2.y) < 1e-6f) {
            continue;
        }
        RansacLine testLine = FitLine(p1, p2);
        testLine.inliers = CountInliers(testLine, points);
        if (testLine.inliers > bestInliers) {
            bestInliers = testLine.inliers;
            bestLine = testLine;
            bestLine.valid = true;
        }
    }
    return bestLine;
}

RansacLine RANSAC::FitLine(const Point& p1, const Point& p2) {
    RansacLine fit;
    fit.m = (p2.x - p1.x) / (p2.y - p1.y);
    fit.b = p1.x - fit.m * p1.y;
    return fit;
}

float RANSAC::CalcDistanceSq(const RansacLine& l, const Point& p) {
    float diff = p.x - (l.m * p.y + l.b);
    return (diff * diff) / (1.0f + l.m * l.m); // no sqrt
}

uint16_t RANSAC::CountInliers(const RansacLine& testLine, const rowPoints& row) {
    uint16_t count = 0;
    for (uint16_t i = 0; i < row.count; i++) {
        Point p = {row.x[i], row.y[i]};
        if (CalcDistanceSq(testLine, p) < RANSAC_DISTANCE_THRESHOLD_SQ) {
            count++;
        }
    }
    return count;
}

SeperatedPoints RANSAC::SeperatePoints(const LidarData& lidar) {
    SeperatedPoints row;
    row.leftPoints.count  = 0;
    row.rightPoints.count = 0;

    int skippedQuality  = 0;
    int skippedDistance = 0;

    for (uint16_t i = 0; i < lidar.count; i++) {
        const LidarPoint& pt = lidar.points[i];

        if (pt.distance == 0) { skippedDistance++; continue; }
        if (pt.quality < LIDAR_MIN_QUALITY) { skippedQuality++; continue; }

        float rad = pt.angle * (M_PI / 180.0f);

        if (pt.angle >= MIN_RIGHT_ANGLE && pt.angle <= MAX_RIGHT_ANGLE) {
            row.leftPoints.x[row.leftPoints.count] = pt.distance * cosf(rad - (M_PI / 2));
            row.leftPoints.y[row.leftPoints.count] = pt.distance * sinf(rad - (M_PI / 2));
            row.leftPoints.count++;
        }
        if (pt.angle > MIN_LEFT_ANGLE && pt.angle <= MAX_LEFT_ANGLE) {
            row.rightPoints.x[row.rightPoints.count] = pt.distance * cosf(rad - (M_PI / 2));
            row.rightPoints.y[row.rightPoints.count] = pt.distance * sinf(rad - (M_PI / 2));
            row.rightPoints.count++;
        }
    }

    // Debug print — remove once issue is found
    Serial.print("[RANSAC] in=");
    Serial.print(lidar.count);
    Serial.print(" skippedDist=");
    Serial.print(skippedDistance);
    Serial.print(" skippedQuality=");
    Serial.print(skippedQuality);
    Serial.print(" leftPts=");
    Serial.print(row.leftPoints.count);
    Serial.print(" rightPts=");
    Serial.println(row.rightPoints.count);

    return row;
}  

bool ValidateScan() {
    return false;
   // rate of change filter?
}
