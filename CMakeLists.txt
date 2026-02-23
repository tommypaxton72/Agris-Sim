set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_C_COMPILER x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_PREFIX_PATH /opt/mingw)
set(SFML_STATIC_LIBRARIES TRUE)
set(SFML_DEPENDENCIES_INSTALL_PREFIX /opt/mingw)

cmake_minimum_required(VERSION 3.10)
project(AgrisSim)
set(CMAKE_CXX_STANDARD 17)

find_package(yaml-cpp REQUIRED)
find_package(SFML 2.6 REQUIRED COMPONENTS graphics window system)

set_property(TARGET OpenGL PROPERTY INTERFACE_LINK_LIBRARIES "opengl32")

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(AgrisSim ${SOURCES})

target_include_directories(AgrisSim PRIVATE 
    include
    /opt/mingw/include
)

target_link_directories(AgrisSim PRIVATE /opt/mingw/lib)

target_link_libraries(AgrisSim
    yaml-cpp::yaml-cpp
    sfml-graphics
    sfml-window
    sfml-system
    opengl32
    winmm
    gdi32
    freetype
    -static -static-libgcc -static-libstdc++
)
