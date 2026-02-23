# Start with a clean ubuntu image
FROM ubuntu:22.04
# Prevent confirmation pop ups?
ENV DEBIAN_FRONTEND=noninteractive

# Get build tools and other things
RUN apt-get update && apt-get install -y \
    build-essential \
    vim \
    git \
    mingw-w64 \
    wget \
    unzip \
    pkg-config \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libgl1-mesa-dev \
    libfreetype6-dev \
    libgles2-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# Set Environment for tool chains
ENV CC=x86_64-w64-mingw32-gcc
ENV CXX=x86_64-w64-mingw32-g++

# Had a problem getting the cmake version needed for SFML
WORKDIR /opt
RUN wget https://github.com/Kitware/CMake/releases/download/v3.28.2/cmake-3.28.2-linux-x86_64.sh && \
    chmod +x cmake-3.28.2-linux-x86_64.sh && \
    ./cmake-3.28.2-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.28.2-linux-x86_64.sh

# Clone dependencies
WORKDIR /deps
RUN git clone https://github.com/jbeder/yaml-cpp.git
RUN git clone --depth 1 --branch 2.6.1 https://github.com/SFML/SFML.git

# Build yaml-cpp
WORKDIR /deps/yaml-cpp
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_SYSTEM_NAME=Windows \
             -DCMAKE_C_COMPILER=$CC \
             -DCMAKE_CXX_COMPILER=$CXX \
             -DCMAKE_CXX_STANDARD=17 \
             -DBUILD_SHARED_LIBS=OFF \
             -DYAML_CPP_BUILD_TESTS=OFF \
             -DYAML_CPP_BUILD_TOOLS=OFF \
             -DCMAKE_INSTALL_PREFIX=/opt/mingw && \
    make -j$(nproc) && make install


# Build SFML
WORKDIR /deps/SFML
RUN apt-get update && apt-get install -y libgl-dev
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_SYSTEM_NAME=Windows \
             -DCMAKE_C_COMPILER=$CC \
             -DCMAKE_CXX_COMPILER=$CXX \
             -DCMAKE_CXX_STANDARD=17 \
             -DBUILD_SHARED_LIBS=OFF \
             -DSFML_BUILD_AUDIO=OFF \
             -DSFML_BUILD_NETWORK=OFF \
             -DSFML_USE_STATIC_STD_LIBS=TRUE \
             -DSFML_DEPENDENCIES_INSTALL_PREFIX=/opt/mingw \
             -DSFML_MISC_INSTALL_PREFIX=/opt/mingw \
             -DCMAKE_INSTALL_PREFIX=/opt/mingw \
             -DSFML_OPENGL_ES=OFF \
             -DSFML_USE_SYSTEM_DEPS=OFF && \
    make -j$(nproc) && make install

# fix for SFML looking at the host OS to determine build
RUN sed -i 's/if (FIND_SFML_OS_WINDOWS)/if (TRUE)/' \
    /opt/mingw/lib/cmake/SFML/SFMLConfigDependencies.cmake
RUN sed -i 's/set(FIND_SFML_OS_LINUX 1)/set(FIND_SFML_OS_LINUX 0)/' \
    /opt/mingw/lib/cmake/SFML/SFMLConfigDependencies.cmake

# Build location
WORKDIR /project/build

CMD ["bash"]
