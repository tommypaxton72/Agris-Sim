cmake_minimum_required(VERSION 3.10)
project(AgrisSim)

set(CMAKE_CXX_STANDARD 17)

find_package(yaml-cpp REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(AgrisSim
    src/main.cpp
    src/world.cpp
    src/robot.cpp
    src/obstacles.cpp
    src/renderer.cpp
    src/controller.cpp
    src/vec.cpp
    src/logger.cpp
)

target_include_directories(AgrisSim PRIVATE include)

target_link_libraries(AgrisSim
    yaml-cpp
    sfml-graphics
    sfml-window
    sfml-system
)
