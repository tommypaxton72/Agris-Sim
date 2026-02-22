#include "renderer.h"
#include "simconfig.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

int main() {

    SimConfig config;
    try {
        YAML::Node node = YAML::LoadFile("config/sim.yaml");
        config.windowWidth  = node["sim"]["windowWidth"].as<int>();
        config.windowHeight = node["sim"]["windowHeight"].as<int>();
        config.windowTitle  = node["sim"]["windowTitle"].as<std::string>();
        config.targetFPS    = node["sim"]["targetFPS"].as<int>();
        config.dt           = node["sim"]["dt"].as<float>();
        config.worldSize    = {
            node["sim"]["worldX"].as<float>(),
            node["sim"]["worldY"].as<float>()
        };
    } catch (const YAML::BadFile& e) {
        std::cerr << "Could not load sim.yaml: " << e.what() << std::endl;
        return -1;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing sim.yaml: " << e.what() << std::endl;
        return -1;
    }

	// Set world size
    World world(config.worldSize);

	// Set Window size
    Renderer renderer(config.windowWidth, config.windowHeight, config.windowTitle);

    
    world.LoadControllerConfig("config/sim.yaml");

    sf::Clock clock;
    // Only one loop
    while (renderer.IsOpen()) {
        float dt = clock.restart().asSeconds();
        renderer.PollEvents();
        world.Update(dt);
        renderer.Draw(world);
    }

    return 0;
}
