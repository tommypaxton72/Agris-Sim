cmake_minimum_required(VERSION 3.10)
project(AgrisSim)

set(CMAKE_CXX_STANDARD 17)

set(SFML_STATIC_LIBRARIES TRUE)

find_package(yaml-cpp REQUIRED)
find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(AgrisSim ${SOURCES})

target_include_directories(AgrisSim PRIVATE include)

target_link_libraries(AgrisSim
    yaml-cpp
    SFML::Graphics
    SFML::Window
    SFML::System
)

# Static build for MinGW
if(MINGW)
    target_link_libraries(AgrisSim -static -static-libgcc -static-libstdc++)
endif()

