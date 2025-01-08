#!/bin/bash



# Build PX4 SITL with Gazebo if not already built
if [ ! -f /root/PX4-Autopilot/build/px4_sitl_default/bin/px4 ]; then
    echo "Building PX4 SITL with Gazebo..."
    source ./PX4-Autopilot/Tools/setup/ubuntu.sh
fi
