#ifndef RANSAC_H
#define RANSAC_H

#include "inocompat.h"
#include <vector>
#include <cmath>
#include <deque>

/*RANSAC DESCRIPTION
STEPS:
1: Convert LiDAR angle and radius to x and y (cartesian conversion)
2: Take two random converted data points. (RANSACloop)
3: Put them into a line equation, normalize it, and see if other points fit.(fitter, pointLine)
4: Finish the loop and find the best line (RANSACloop)
4: Calculate the distance from the car to the calculated inlier line. (distancetoLine)
5: Calculate heading angle and error (headingAngle)
6: Validate the line for the state-machine (lineValidation)
*/

//Going to change this soon
const int MAX_ITERATIONS = 400;
//Everything within a 3 inch thresh-hold of the line will be considered as an inlier
const float DIST_THRESHOLD = 30; // 75mm = 2.95 inches. 
const int MIN_INLIERS = 30; 

struct points {
  float x;
  float y;
  bool isValid;
};

struct plane {
  float a;
  float b;
  float c;
};

class RANSAC {

  public: 
    void cartesianConversion(const std::deque<float>& angles, const std::deque<float>& ranges, std::vector<points>& points);
    plane fitter(const points& p1, const points& p2);
    float pointLine(const plane& line, const points& p);
    void RANSACLoop(const std::vector<points>& points);
    void distancetoLine();
    void headingAngle(float targetDistance);
    bool lineValidation();

    int bestInlierCount = 0;
    float distance = 0.0f;
    float headingError = 0.0f;
    float distanceError = 0.0f;
    plane bestLine = {0.0f, 0.0f, 0.0f};
  
  private:
    plane currentLine;
};

#endif
