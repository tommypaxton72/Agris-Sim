#ifndef RENDERER_H
#define RENDERER_H

#include <SFML/Graphics.hpp>
#include "world.h"      // gets Robot, Obstacles, pose for free

class Renderer {
public:
    Renderer(int windowWidth, int windowHeight, const std::string& title, float worldW, float worldH);
    
    // Main draw call - takes const refs so renderer cant modify simulation
    void Draw(const World& world);
    
    // Returns false when window is closed
    bool IsOpen();
    
    // Handle window events like closing
    void PollEvents();

	void LoadControllerConfig(const std::string& configPath);

private:
    sf::RenderWindow window;
	sf::View worldView;
    // Individual draw helpers
    void DrawRobot(const pose& p, const robo& r);
    void DrawObstacles(const std::vector<Obstacle>& obstacles);
    void DrawWorld(const WorldSize& size);
};













#endif
