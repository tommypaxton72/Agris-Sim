#!/bin/bash

set -e

OUTPUT_DIR="/mnt/c/Users/Tommy/Documents/Projects/AgrisSim"

echo "Starting build..."

docker run --rm \
    -v "$(pwd)":/project \
    -w /project/build \
    agrissim:latest \
    bash -c "cmake .. && make -j$(nproc)" # -j$(nproc) uses all available CPU cores for faster compilation.

echo "Build succeeded, copying .exe to Windows filesystem..."

# Copy everything in build/ matching *.exe to the Windows output folder
# Adjust the source path if your .exe lands somewhere other than build/
cp "$(pwd)"/build/*.exe "$OUTPUT_DIR"

echo "Done. Output: $OUTPUT_DIR"