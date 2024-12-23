#!/bin/bash

# Debug: Log the input arguments
echo "Running entrypoint.sh with arguments: $@" > /tmp/entrypoint.log

# Ensure the runtime directory exists with the correct permissions
mkdir -p /tmp/runtime-docker
chmod 0700 /tmp/runtime-docker

# Debug: Check if runtime directory exists and permissions
ls -ld /tmp/runtime-docker >> /tmp/entrypoint.log

# Adding local model path to the Gazebo sim
export GZ_SIM_RESOURCE_PATH=/root/ros2/local_models/

# Check for the -f flag
FORCE_INSTALL=false
if [[ "$*" == *"-f"* ]]; then
    FORCE_INSTALL=true
    echo "Force install enabled (-f flag detected). Installing ros2_controller in src..." >> /tmp/entrypoint.log
fi

# Ensure ros2 workspace exists
mkdir -p ~/ros2/src
cd ~/ros2/

# Adding support for ros2_control
if [ "$FORCE_INSTALL" = true ] || [ ! -d src/ros-controls ]; then
    echo "Setting up ros2_controller in src..." >> /tmp/entrypoint.log
    vcs import --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.$ROS_DISTRO.repos src
    rosdep update --rosdistro=$ROS_DISTRO
    sudo apt-get update
    rosdep install --from-paths src --ignore-src -r -y
    . /opt/ros/${ROS_DISTRO}/setup.sh
    colcon build --symlink-install
else
    echo "ros2_controller already exists in src. Skipping import and build." >> /tmp/entrypoint.log
fi

# Debug: Log the directory structure of ros2
ls -l ~/ros2 >> /tmp/entrypoint.log

# Source the ROS 2 setup
. ~/ros2/install/setup.bash


# Execute the main container command
exec "$@"

