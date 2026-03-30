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
    const Debug& db = world.GetDebug();
	
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
    switch (db.state) {
        case 1: {
            DrawRANSACLine(p, world.GetRightLine(),
                world.GetRightLine().valid ? sf::Color(0, 255, 0, 200) : sf::Color(255, 0, 0, 120));
            DrawRANSACLine(p, world.GetLeftLine(),
                world.GetLeftLine().valid  ? sf::Color(0, 255, 0, 200) : sf::Color(255, 0, 0, 120));
            break;
            };
        case 2: {
            DrawRANSACLine(p, db.lineEOR,
                db.lineEOR.valid ? sf::Color(255, 165, 0, 200) : sf::Color(255, 0, 0, 120));
            break;
        };
        default:
            break;
    }

    DrawWaypoints(db);
	DrawData(db);

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

// Lidar gets set to zero if its above the maximum so it only draws the lines that hit an object.
void Renderer::DrawLidar(const pose& p, const LidarData& data) {
	int N = 15;
    for (int i = 0; i < data.count; i += N) {
        sf::VertexArray ray(sf::Lines, 2);
        
        // Ray starts at robot center
        ray[0].position = sf::Vector2f(p.x, p.y);
        // Lidar is stored as degrees needs to convert to rads
        float angleRad = data.points[i].angle * (M_PI / 180.0f);

        ray[1].position = sf::Vector2f(
        p.x + data.points[i].distance * std::cos(p.theta - angleRad),
        p.y + data.points[i].distance * std::sin(p.theta - angleRad));
        
        ray[0].color = sf::Color(255, 255, 0, 100); // yellow, semi-transparent
        ray[1].color = sf::Color(255, 255, 0, 100);

        window.draw(ray);
    }
}
void Renderer::DrawRANSACLine(const pose& p, const RansacLine& line, sf::Color color) {
    // Don't draw if RANSAC hasn't found a line yet
    if (!line.valid) return;

    // Line equation is x = m*y + b in robot-centred coordinates.
    // y is the independent variable because crop rows run parallel to the
    // robot's forward axis — this avoids vertical line singularity.
    // Pick two y values along the forward axis and solve for x.
    const float span = 3000.0f;

    float y1 = -span;
    float y2 =  span;
    float x1 = line.m * y1 + line.b;
    float x2 = line.m * y2 + line.b;

    float cos = std::cos(p.theta);
    float sin = std::sin(p.theta);

    // RANSAC frame: x=lateral(left+), y=longitudinal(backward+, forward-)
    // Robot-standard: forward=-y_ransac, left=+x_ransac
    // World transform: wx = -sin*x - cos*y,  wy = cos*x - sin*y
    float wx1 =  sin * x1 - cos * y1;
    float wy1 = -cos * x1 - sin * y1;
    float wx2 =  sin * x2 - cos * y2;
    float wy2 = -cos * x2 - sin * y2;
    
    sf::VertexArray lineShape(sf::Lines, 2);
    lineShape[0].position = sf::Vector2f(p.x + wx1, p.y + wy1);
    lineShape[1].position = sf::Vector2f(p.x + wx2, p.y + wy2);
    lineShape[0].color    = color;
    lineShape[1].color    = color;

    window.draw(lineShape);
}

// This needs to be changed for the new states
static const char* StateToString(int state) {
    switch (state) {
        case 0: return "STOP";
        case 1: return "INBETWEEN_ROWS";
        case 2: return "END_OF_ROW";
        case 3: return "TURNING";
        case 4: return "ALIGNING";
        case 5: return "ESTOP";
        case 6: return "COLLISION_AVOIDANCE";
        default: return "UNKNOWN";
    }
}

// Might add timer to slow down the writing of data to hud
void Renderer::DrawData(const Debug& debug) {
	window.setView(window.getDefaultView());

    sf::Text leftPWM;
    sf::Text rightPWM;
    sf::Text leftDistance;
	sf::Text rightDistance;
    sf::Text lineDifference;
	sf::Text state;
    sf::Text lWaypoint;
    sf::Text gWaypoint;

    leftPWM.setFont(font);
    rightPWM.setFont(font);
	leftDistance.setFont(font);
    rightDistance.setFont(font);
    lineDifference.setFont(font);
	state.setFont(font);
    lWaypoint.setFont(font);
    gWaypoint.setFont(font);

    auto fmt = [](float v) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%+.2f", v);
        return std::string(buf);
    };

	// Left Motor PWM
	leftPWM.setCharacterSize(16);
    leftPWM.setFillColor(sf::Color::White);
    leftPWM.setPosition(20.0f, 20.0f);
    leftPWM.setString("L: " + fmt(debug.motor.leftMotor.PWM));

	// Right Motor PWM
    rightPWM.setCharacterSize(16);
    rightPWM.setFillColor(sf::Color::White);
    rightPWM.setPosition(20.0f, 45.0f);
    rightPWM.setString("R: " + fmt(debug.motor.rightMotor.PWM));

    // Left Wall
	leftDistance.setCharacterSize(16);
    leftDistance.setFillColor(sf::Color::White);
    leftDistance.setPosition(20.0f, 70.0f);
    leftDistance.setString("Left Wall: " + fmt(debug.RansacLines.leftLine.b));

    // Right Wall
    rightDistance.setCharacterSize(16);
    rightDistance.setFillColor(sf::Color::White);
    rightDistance.setPosition(20.0f, 95.0f);
    rightDistance.setString("Right Wall: " + fmt(debug.RansacLines.rightLine.b));

    // Line difference
    lineDifference.setCharacterSize(16);
    lineDifference.setFillColor(sf::Color::White);
    lineDifference.setPosition(20.0f, 120.0f);
    lineDifference.setString("Diff: " + fmt(debug.RansacLines.leftLine.b - debug.RansacLines.rightLine.b));

    // Current state
    state.setCharacterSize(16);
    state.setFillColor(sf::Color::Blue);
	state.setPosition(20.0f, 145.0f);
	state.setString("State: " + std::string(StateToString(debug.state)));

    // Local Waypoint
    lWaypoint.setCharacterSize(16);
    lWaypoint.setFillColor(sf::Color::Cyan);
    lWaypoint.setPosition(20.0f, 170.0f);
    lWaypoint.setString("Local WP: " + fmt(debug.lWaypoint.x) + ", " + fmt(debug.lWaypoint.y));

    // Global Waypoint
    gWaypoint.setCharacterSize(16);
    gWaypoint.setFillColor(sf::Color::Cyan);
    gWaypoint.setPosition(20.0f, 195.0f);
    const Waypoint& activeWP = debug.gWaypoint[debug.currentWaypointIndex];
    gWaypoint.setString("Global WP: " + fmt(activeWP.x) + ", " + fmt(activeWP.y));

    window.draw(leftPWM);
    window.draw(rightPWM);
    window.draw(leftDistance);
	window.draw(rightDistance);
    window.draw(lineDifference);
	window.draw(state);
    window.draw(lWaypoint);
    window.draw(gWaypoint);
	window.setView(worldView);
}

void Renderer::DrawWaypoints(const Debug& debug) {
    const float radius = 30.0f;
    for (int i = 0; i < MAX_WAYPOINTS; i++) {
        const Waypoint& wp = debug.gWaypoint[i];
        if (!wp.valid) continue;
        sf::CircleShape dot(radius);
        dot.setOrigin(radius, radius);
        dot.setPosition(wp.x, wp.y);
        dot.setFillColor(i == debug.currentWaypointIndex
            ? sf::Color(255, 255, 0, 220)
            : sf::Color(0, 200, 255, 150));
        window.draw(dot);
    }
}
