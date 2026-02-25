#ifndef RENDERER_H
#define RENDERER_H

#include <SFML/Graphics.hpp>
#include "world.h"      // gets Robot, Obstacles, pose for free
#include "types.h"
#include "datalayer.h"

class Renderer {
public:
    Renderer(int windowWidth, int windowHeight, const std::string& title, float worldW, float worldH, bool Lidar);
    
    // Main draw
    void Draw(const World& world);
    bool IsOpen();
    void PollEvents();
	
private:
    sf::RenderWindow window;
    sf::View worldView;

	bool showLidar = 0;
    // Individual draw helpers
    void DrawRobot(const pose& p, const robo& r);
    void DrawObstacles(const std::vector<Obstacle>& obstacles);
    void DrawWorld(const WorldSize& size);
	void DrawLidar(const pose& p, const LidarData& data);
};













#endif
