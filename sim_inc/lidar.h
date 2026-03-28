#ifndef LIDAR_H
#define LIDAR_H

#include <vector>
#include "types.h"
#include "datastructs.h"



class Lidar {
  public:
    Lidar(int inNumofRays, float maxDist);
	  void SetConfig(int rays, float maxDist);
	
    LidarData GetScan(const pose& p, const std::vector<Obstacle>& obstacle);
  private:
    int numofRays = 0;
    float maxDistance = 0;

	  float CastRay(const pose& p, const std::vector<Obstacle>& obstacles, float angle);
	  std::vector<Obstacle> CheckObstacles(const pose& p, const std::vector<Obstacle>& obstacle);
    };    


#endif
