#!/bin/bash

set -e

WIN_USER=$(cmd.exe /c "echo %USERNAME%" 2>/dev/null | tr -d '\r')  # get Windows username at runtime
OUTPUT_DIR="/mnt/c/Users/$WIN_USER/Documents/Projects/AgrisSim"

echo "Starting build..."

docker run --rm \
    -v "$(pwd)":/project \
    -w /project/build \
    agrissim:latest \
    bash -c "rm -rf * && cmake .. && make -j$(nproc)"

echo "Build succeeded, copying .exe to Windows filesystem..."

cp "$(pwd)"/build/*.exe "$OUTPUT_DIR"

echo "Done. Output: $OUTPUT_DIR"