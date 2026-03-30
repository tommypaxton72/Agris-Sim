#ifndef RENDERER_H
#define RENDERER_H

#include <SFML/Graphics.hpp>
#include "world.h"      // gets Robot, Obstacles, pose for free
#include "types.h"
#include "datastructs.h"

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
	sf::Font font;
	bool showLidar = 0;
    // Individual draw helpers
    void DrawRobot(const pose& p, const robo& r);
    void DrawObstacles(const std::vector<Obstacle>& obstacles);
    void DrawWorld(const WorldSize& size);
    void DrawLidar(const pose& p, const LidarData& data);
	// Store world dimensions so we can recalculate the view on any resize
    float worldW = 0.0f;
    float worldH = 0.0f;
	void DrawRANSACLine(const pose& p, const RansacLine& line, sf::Color color);
    void DrawWaypoints(const Debug& debug);
    void UpdateView(unsigned int windowW, unsigned int windowH);
	void DrawData(const Debug& debug);
    };













#endif
