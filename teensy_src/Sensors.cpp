#include "Sensors.h"

void sensors::calibrateIMU(uint8_t rawZ){
  int sum = 0;
  int count = 0;

  sum += rawZ;
  count++;
  
  if(count > 0){
    biasZ = (float)sum / count;
  }
}

void sensors::process(uint8_t rawZ){
  float centeredZ = (float)rawZ - biasZ;

  filteredGyroZ = (alpha * centeredZ) + ((1.0 - alpha) * filteredGyroZ);
}