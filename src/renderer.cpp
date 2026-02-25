#include "renderer.h"
#include <cmath>


// Constructor - create the window
Renderer::Renderer(int windowWidth, int windowHeight, const std::string& title, float worldW, float worldH, bool Lidar) {
    window.create(sf::VideoMode(windowWidth, windowHeight), title);
    window.setFramerateLimit(60);
	showLidar = Lidar;
    worldView = sf::View(sf::FloatRect(0, 0, worldW, worldH));
    window.setView(worldView);
}

// Check if window is still open
bool Renderer::IsOpen() {
    return window.isOpen();
}

// Handle window events like clicking the close button
void Renderer::PollEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        }
    }
}

// Main draw call - clears, draws everything, displays
void Renderer::Draw(const World& world) {
    const pose& p = world.GetRobotPose();
	
    worldView.setCenter(p.x, p.y);
    window.setView(worldView);
    
    window.clear(sf::Color(50, 50, 50));
    DrawWorld(world.GetWorldSize());
    DrawObstacles(world.GetObstacles());
    DrawRobot(p, world.GetRobotConfig());
	if (showLidar == false) {
		DrawLidar(p, world.GetLidarData());
	}
    window.display();
}

// Draw world boundary as a rectangle outline
void Renderer::DrawWorld(const WorldSize& size) {
    sf::RectangleShape boundary(sf::Vector2f(size.x, size.y));
    // No fill, just outline so we can see the world bounds
    boundary.setFillColor(sf::Color::Transparent);
    boundary.setOutlineColor(sf::Color::White);
    boundary.setOutlineThickness(2.0f);
    window.draw(boundary);
}

// Draw each obstacle as a circle
void Renderer::DrawObstacles(const std::vector<Obstacle>& obstacles) {
    for (const auto& obs : obstacles) {
        sf::CircleShape circle(obs.radius);
        // Center the circle on the obstacle position
        circle.setOrigin(obs.radius, obs.radius);
        circle.setPosition(obs.x, obs.y);
        circle.setFillColor(sf::Color(200, 50, 50));  // red
        window.draw(circle);
    }
}

// Draw robot as a circle with a line showing heading
void Renderer::DrawRobot(const pose& p, const robo& r) {
    // Create rectangle using actual robot dimensions from yaml
    sf::RectangleShape body(sf::Vector2f(r.length, r.width));
    
    // Set origin to center so rotation works correctly around robot center
    body.setOrigin(r.length / 2.0f, r.width / 2.0f);
    body.setPosition(p.x, p.y);
    
    // SFML uses degrees, pose.theta is radians so convert
    body.setRotation(p.theta * (180.0f / M_PI));
    body.setFillColor(sf::Color(50, 200, 50));  // green

    // Heading line from center to front of robot
    sf::VertexArray heading(sf::Lines, 2);
    heading[0].position = sf::Vector2f(p.x, p.y);
    heading[1].position = sf::Vector2f(
        p.x + (r.length / 2.0f) * std::cos(p.theta),
        p.y + (r.length / 2.0f) * std::sin(p.theta));
    heading[0].color = sf::Color::White;
    heading[1].color = sf::Color::White;

    window.draw(body);
    window.draw(heading);
}

void Renderer::DrawLidar(const pose& p, const LidarData& data) {
    for (int i = 0; i < data.count; i++) {
        sf::VertexArray ray(sf::Lines, 2);
        
        // Ray starts at robot center
        ray[0].position = sf::Vector2f(p.x, p.y);
        
        // Ray end point calculated from angle and distance
        ray[1].position = sf::Vector2f(
            p.x + data.points[i].distance * std::cos(data.points[i].angle),
            p.y + data.points[i].distance * std::sin(data.points[i].angle));
        
        ray[0].color = sf::Color(255, 255, 0, 100); // yellow, semi-transparent
        ray[1].color = sf::Color(255, 255, 0, 100);
        
        window.draw(ray);
    }
}
