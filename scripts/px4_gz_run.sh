#!/bin/bash

# Default target
TARGET=${1:-gz_x500}

# Navigate to PX4-Autopilot directory
cd ./PX4-Autopilot || { echo "PX4-Autopilot directory not found!"; exit 1; }

# Clean previous builds
echo "Cleaning previous build..."
make clean

# Build PX4 SITL with specified target
echo "Building PX4 SITL with target: $TARGET"
make -j16 px4_sitl $TARGET

