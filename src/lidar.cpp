#include "lidar.h"
#include <cmath>

Lidar::Lidar(int inNumofRays, float maxDist) {
    numofRays = inNumofRays;
	maxDistance = maxDist;
}

void Lidar::SetConfig(int rays, float maxDist) {
    numofRays = rays;
    maxDistance = maxDist;
}

LidarData Lidar::GetScan(const pose& p, const std::vector<Obstacle>& obstacle) {
    LidarData data;
    std::vector<Obstacle> nearby = CheckObstacles(p, obstacle);
    float angleStep = (2.0f * M_PI) / numofRays;
	
    for (int i = 0; i < numofRays; i++) {
        data.points[i].angle = p.theta + i * angleStep;
        data.points[i].distance = CastRay(p, nearby, data.points[i].angle);
    }
	data.count = numofRays;
	return data;
}

float Lidar::CastRay(const pose& p, const std::vector<Obstacle>& obstacles, float angle) {
    float dx = std::cos(angle);
    float dy = std::sin(angle);
    float closest = maxDistance;

    for (const auto& obs : obstacles) {
        
        float fx = p.x - obs.x;
        float fy = p.y - obs.y;

        float a = dx*dx + dy*dy;
        float b = 2.0f * (fx*dx + fy*dy);
        float c = fx*fx + fy*fy - obs.radius*obs.radius;
        float discriminant = b*b - 4*a*c;

        if (discriminant >= 0) {
            float t = (-b - std::sqrt(discriminant)) / (2.0f * a);
            if (t > 0 && t < closest) {
                closest = t;
            }
        }
    }
    return closest;
}

std::vector<Obstacle> Lidar::CheckObstacles(const pose& p, const std::vector<Obstacle>& obstacle) {
	std::vector<Obstacle> nearby;
    for (const auto& obs : obstacle) {
		float dx = p.x - obs.x;
        float dy = p.y - obs.y;
        float d = dx * dx + dy * dy;
        if (d <= maxDistance * maxDistance) {
            nearby.push_back(obs);
        }
	}
    return nearby;
}
