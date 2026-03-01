#include "renderer.h"
#include <cmath>


// Constructor - create the window
Renderer::Renderer(int windowWidth, int windowHeight, const std::string& title, float worldW, float worldH, bool Lidar) {
    window.create(sf::VideoMode(windowWidth, windowHeight), title);
    window.setFramerateLimit(60);
	showLidar = Lidar;
	// Store world dimensions so UpdateView can reference them later
    this->worldW = worldW;
    this->worldH = worldH;
    worldView = sf::View(sf::FloatRect(0, worldH, worldW, -worldH));
    window.setView(worldView);
    // Run UpdateView once at startup so initial sizing is correct
    UpdateView(windowWidth, windowHeight);
	if (!font.loadFromFile("JetBrainsMono-light.ttf")) {
        std::cerr << "No font loaded!!" << std::endl;
	}
    
}

void Renderer::UpdateView(unsigned int windowWidth, unsigned int windowHeight) {
    // Calculate the aspect ratios of both the world and the current window
    float worldRatio  = worldW / worldH;
    float windowRatio = (float)windowWidth / (float)windowHeight;

    // Viewport is in 0.0-1.0 normalized coordinates, representing
    // what portion of the window the view occupies
    float viewportX = 0.0f;
    float viewportY = 0.0f;
    float viewportW = 1.0f;
    float viewportH = 1.0f;

    if (windowRatio > worldRatio) {
        // Window is wider than the world ratio - add side bars
        // Scale width down so the world fits height-wise, then center it
        viewportW = worldRatio / windowRatio;
        viewportX = (1.0f - viewportW) / 2.0f;
    } else {
        // Window is taller than the world ratio - add top/bottom bars
        // Scale height down so the world fits width-wise, then center it
        viewportH = windowRatio / worldRatio;
        viewportY = (1.0f - viewportH) / 2.0f;
    }

    worldView.setViewport(sf::FloatRect(viewportX, viewportY, viewportW, viewportH));
    window.setView(worldView);
}


// Check if window is still open
bool Renderer::IsOpen() {
    return window.isOpen();
}

void Renderer::PollEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        }
        // SFML fires this event any time the user resizes the window
        if (event.type == sf::Event::Resized) {
            // Recalculate viewport so world stays correctly proportioned
            UpdateView(event.size.width, event.size.height);
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
	if (showLidar == true) {
		DrawLidar(p, world.GetLidarData());
    }
	// Draw RANSAC lines on top of everything else so they're always visible.
    // Green = valid line used for steering, red = invalid/not enough inliers.
    DrawRANSACLine(p, world.GetRightLine(),
        world.GetRightLine().valid ? sf::Color(0, 255, 0, 200) : sf::Color(255, 0, 0, 120));
    DrawRANSACLine(p, world.GetLeftLine(),
        world.GetLeftLine().valid  ? sf::Color(0, 255, 0, 200) : sf::Color(255, 0, 0, 120));


	DrawData(world.GetDataLayer());

    window.display();


    
    }

// Draw world boundary as a rectangle outline
void Renderer::DrawWorld(const WorldSize& size) {
    sf::RectangleShape boundary(sf::Vector2f(size.x, size.y));
    // No fill, just outline so we can see the world bounds
    boundary.setFillColor(sf::Color::Transparent);
    boundary.setOutlineColor(sf::Color::White);
    boundary.setOutlineThickness(-10.0f);
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

// Draw robot as a square with a line showing heading
void Renderer::DrawRobot(const pose& p, const robo& r) {
    
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
	int N = 15;
    for (int i = 0; i < data.count; i += N) {
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
void Renderer::DrawRANSACLine(const pose& p, const RANSACLine& line, sf::Color color) {
    // Don't draw if RANSAC hasn't found anything yet — all zeros means no data
    if (line.a == 0.0f && line.b == 0.0f && line.c == 0.0f) return;

    // The line equation is Ax + By + C = 0 in robot-centred coordinates.
    // To find two drawable points we pick two x values either side of the
    // robot and solve for y: y = -(Ax + C) / B
    // Then add the robot's world position to move from robot space to world space.
    // We use a large span (3000mm either side) so the line extends well past
    // the visible obstacles.

    const float span = 3000.0f;

    float x1 = -span;
    float x2 =  span;

    float y1, y2;

    if (std::fabs(line.b) > 0.0001f) {
        // Normal case — solve for y given x
        y1 = -(line.a * x1 + line.c) / line.b;
        y2 = -(line.a * x2 + line.c) / line.b;
    } else {
        // Line is nearly vertical — solve for x given y instead
        // Ax + C = 0  →  x = -C/A
        x1 = -line.c / line.a;
        x2 = x1;
        y1 = -span;
        y2 =  span;
    }

    // Convert from robot-centred to world coordinates by adding robot position
    sf::VertexArray lineShape(sf::Lines, 2);
    lineShape[0].position = sf::Vector2f(p.x + x1, p.y + y1);
    lineShape[1].position = sf::Vector2f(p.x + x2, p.y + y2);
    lineShape[0].color    = color;
    lineShape[1].color    = color;

    window.draw(lineShape);
}

// Converts state int back to a readable string for display
static const char* StateToString(int state) {
    switch (state) {
        case 0: return "STOP";
        case 1: return "INBETWEEN_ROWS";
        case 2: return "SEARCHING_FOR_WALLS";
        default: return "UNKNOWN";
    }
}

void Renderer::DrawData(const DataLayer& dataLayer) {
	window.setView(window.getDefaultView());

    sf::Text leftPWM;
    sf::Text rightPWM;
    sf::Text leftDistance;
	sf::Text rightDistance;
    sf::Text lineDifference;
    sf::Text zRate;
	sf::Text PIDResult;
	sf::Text state;

    leftPWM.setFont(font);
    rightPWM.setFont(font);
	leftDistance.setFont(font);
    rightDistance.setFont(font);
    lineDifference.setFont(font);
    zRate.setFont(font);
    PIDResult.setFont(font);
	state.setFont(font);

    auto fmt = [](float v) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%+.2f", v);
        return std::string(buf);
    };

    
	// Left Motor
	leftPWM.setCharacterSize(16);
    leftPWM.setFillColor(sf::Color::White);
    leftPWM.setPosition(20.0f, 20.0f);
    leftPWM.setString("L: " + fmt(dataLayer.leftMotor.PWM));

	// Right Motor
    rightPWM.setCharacterSize(16);
    rightPWM.setFillColor(sf::Color::White);
    rightPWM.setPosition(20.0f, 45.0f);
    rightPWM.setString("R: " + fmt(dataLayer.rightMotor.PWM));

    // Left Wall
	leftDistance.setCharacterSize(16);
    leftDistance.setFillColor(sf::Color::White);
    leftDistance.setPosition(20.0f, 70.0f);
    leftDistance.setString("Left Wall: " + fmt(dataLayer.debug.leftDistance));

    // Right Wall
    rightDistance.setCharacterSize(16);
    rightDistance.setFillColor(sf::Color::White);
    rightDistance.setPosition(20.0f, 95.0f);
    rightDistance.setString("Right Wall: " + fmt(dataLayer.debug.rightDistance));
    
    // Line difference
    lineDifference.setCharacterSize(16);
    lineDifference.setFillColor(sf::Color::White);
    lineDifference.setPosition(20.0f, 120.0f);
    lineDifference.setString("Diff: " + fmt(dataLayer.debug.lineDifference));
    
    // zRate
    zRate.setCharacterSize(16);
    zRate.setFillColor(sf::Color::White);
    zRate.setPosition(20.0f, 145.0f);
    zRate.setString("Z: " + fmt(dataLayer.debug.zRate));

    // PID Results
    PIDResult.setCharacterSize(16);
    PIDResult.setFillColor(sf::Color::White);
    PIDResult.setPosition(20.0f, 170.0f);
    PIDResult.setString("PID: " + fmt(dataLayer.debug.PIDResult));

    // Current state
    state.setCharacterSize(16);
    state.setFillColor(sf::Color::Blue);
	state.setPosition(20.0f, 195.0f);
	state.setString("State: " + std::string(StateToString(dataLayer.debug.state)));   
    
    
    window.draw(leftPWM);
    window.draw(rightPWM);
    window.draw(leftDistance);
	window.draw(rightDistance);
    window.draw(lineDifference);
    window.draw(zRate);
    window.draw(PIDResult);
	window.draw(state);
    
	window.setView(worldView);
}


