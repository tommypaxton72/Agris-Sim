# Start with clean ubuntu install
FROM ubuntu:22.04

# Turn off accept messages
ENV DEBIAN_FRONTEND=noninteractive

# Build tools etc
WORKDIR /build
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    mingw-w64 \
    wget \
    unzip \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Set default compiler
ENV CC=x86_64-w64-mingw32-gcc
ENV CXX=x86_64-w64-mingw32-g++

# Clone Dependencies
WORKDIR /deps
RUN git clone https://github.com/jbeder/yaml-cpp.git
RUN git clone https://github.com/SFML/SFML.git

# Build yaml-cpp
WORKDIR /deps/yaml-cpp
RUN mkdir build && cd build && \
    cmake .. \
      -DCMAKE_SYSTEM_NAME=Windows \
      -DCMAKE_C_COMPILER=$CC \
      -DCMAKE_CXX_COMPILER=$CXX \
      -DBUILD_SHARED_LIBS=OFF \
      -DYAML_BUILD_SHARED_LIBS=OFF \
      -DCMAKE_INSTALL_PREFIX=/opt/mingw \
      && make -j$(nproc) \
      && make install

# Build SFML
WORKDIR /deps/SFML
RUN mkdir build && cd build && \
    cmake .. \
      -DCMAKE_SYSTEM_NAME=Windows \
      -DCMAKE_C_COMPILER=$CC \
      -DCMAKE_CXX_COMPILER=$CXX \
      -DBUILD_SHARED_LIBS=OFF \
      -DSFML_BUILD_AUDIO=OFF \
      -DSFML_BUILD_NETWORK=OFF \
      -DSFML_USE_STATIC_STD_LIBS=TRUE \
      -DCMAKE_INSTALL_PREFIX=/opt/mingw && \
    make -j$(nproc) && make install

# Build Project
RUN mkdir -p build
WORKDIR /project/build
RUN cmake .. -DCMAKE_SYSTEM_NAME=Windows \
             -DCMAKE_C_COMPILER=$CC \
             -DCMAKE_CXX_COMPILER=$CXX \
             -DCMAKE_PREFIX_PATH=/opt/mingw \
             -DBUILD_SHARED_LIBS=OFF && \
    make -j$(nproc)

# Create directory for output and mount it to server
RUN cp AgrisSim.exe /project/


WORKDIR /project

CMD ["bash"]
