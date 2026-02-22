set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_C_COMPILER x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)

cmake_minimum_required(VERSION 3.10)
project(AgrisSim)
set(CMAKE_CXX_STANDARD 17)

find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(AgrisSim ${SOURCES})

target_include_directories(AgrisSim PRIVATE 
    include
    /opt/mingw/include
)

target_link_directories(AgrisSim PRIVATE /opt/mingw/lib)

target_link_libraries(AgrisSim
    yaml-cpp::yaml-cpp
    sfml-graphics-s
    sfml-window-s
    sfml-system-s
    opengl32
    winmm
    gdi32
    freetype
    -static -static-libgcc -static-libstdc++
)
