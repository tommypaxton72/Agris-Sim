#ifndef SENSORS_H
#define SENSORS_H

#include <LSM6.h>

class sensors {
  public:

  void calibrateIMU(uint8_t rawZ);

  void process(uint8_t rawZ);

  //We focus on Z, as it represents the 'yaw' in a gyroscope, aka left and right.
  float biasZ = 0;
  float filteredGyroZ = 0;
  const float alpha = 0.15;

  private:


};

#endif