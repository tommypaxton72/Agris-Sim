#ifndef LIDAR_H
#define LIDAR_H

#include "types.h"
#include "datalayer.h"


class Lidar {
  public:
    Lidar();

    LidarData Lidar::GetScan(pose p, const std::vector<Obstacle>& obstacle);
  private:
    int NumofRays;
    float maxDistance;

    LidarData lidarData[1000];

	float CastRay(const pose& p, const std::vector<Obstacle>& obstacles, float angle);
	std::vector<Obstacle> Lidar::CheckObstacles(pose p, const std::vector<Obstacle>& obstacle);
    };    


#endif
