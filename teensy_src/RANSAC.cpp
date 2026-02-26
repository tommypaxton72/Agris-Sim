#include "RANSAC.h"

//Converts the distance and angle (r and thada) to x and y to be used in a 2-D plane.
void RANSAC::cartesianConversion(const std::deque<float>& angles, const std::deque<float>& distances, std::vector<points>& point){
  point.clear();
  points p;

  for(size_t i = 0; i < angles.size(); i++){
    float r = distances[i];

    //sine apparently needs radians instead of degrees to function properly
    float o = angles[i] * (M_PI / 180.0f);

    if (r <= 50.0f)
      continue;
    
    p.x = r * cos(o);
    p.y = r * sin(o);
    point.push_back(p);
  }
}

//Fitter steps...
//A = y2 - y1
//B = x1 - x2
//C = -(A*x1 + B * y1)
//Normalize through sqrt(A^2 + B^2). Do this to make the magnitude of the normal vector.
//Return and divide by normalized if greater than 0.
plane RANSAC::fitter(const points& p1, const points& p2){
  plane currentLine;
  
  currentLine.a = p2.y - p1.y;
  currentLine.b = p1.x - p2.x;
  currentLine.c = -(currentLine.a * p1.x + currentLine.b * p1.y);

  float normalized = sqrt(currentLine.a * currentLine.a + currentLine.b * currentLine.b);

  //Avoid division by 0
  if(normalized > 0){
    currentLine.a /= normalized;
    currentLine.b /= normalized;
    currentLine.c /= normalized;
  }

  return currentLine;
}

//From the equation A*x + B*y + C, or a straight 2-D line equation
//Filters out the outliers in the loop
float RANSAC::pointLine(const plane& line, const points& p){
  return fabs(line.a * p.x + line.b * p.y + line.c);
}

void RANSAC::RANSACLoop(const std::vector<points>& points) {
  bestInlierCount = 0;

  //Resetting best line each time
  bestLine.a = 0;
  bestLine.b = 0;
  bestLine.c = 0;
  distance = 0; 

  if (points.size() < 2)
    return;

  for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
    int i1 = random(0, points.size());
    int i2 = random(0, points.size());

    if (i1 == i2)
      continue;

    currentLine = fitter(points[i1], points[i2]);

    int inlierCount = 0;

    for (const auto& p : points) {
      float d = pointLine(currentLine, p);
      if (d < DIST_THRESHOLD)
        inlierCount++;
    }

    if (inlierCount > bestInlierCount) {
    bestInlierCount = inlierCount;
    
    //Makes sure the math is consistent. Without it, a likes to flip between negative and positive.
      if (currentLine.b < 0) { 
        bestLine.a = -currentLine.a;
        bestLine.b = -currentLine.b;
        bestLine.c = -currentLine.c;
      } 
      else 
        bestLine = currentLine;
    }
  }
}

//Computes distance from the origin (Which is the LiDAR) to the line.
//Formula is d = |C|/sqr(A^2 + B^2)
void RANSAC::distancetoLine(){
  distance = fabs(bestLine.c);
}

//Gonna need editing, probably will use it in tandem with the PID for redundancy
void RANSAC::headingAngle(float targetDistance){
  headingError = atan2(bestLine.a, -bestLine.b);
  distanceError = targetDistance - distance;
}

//Check if line has enough inliers, if its within distance, and if the angle isn't unreasonable.
bool RANSAC::lineValidation(){
  if(bestInlierCount < MIN_INLIERS){
    return false;
  }
  else if(abs(bestLine.c) > 800){ //If the distance from the car to the wall is > 15 inches, the wall isn't correct
    return false;
  }
  else if(headingError > 0.785f){ //Since normalizing the line put it from 0 to 1, we're using radians. 0.785 = 45 degrees
    return false;
  }
  return true;
}




