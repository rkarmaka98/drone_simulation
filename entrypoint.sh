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
    echo "Force install enabled (-f flag detected). Purging ~/ros2/src_con directory..." >> /tmp/entrypoint.log
    rm -rf ~/ros2/src_con
fi

# Ensure ros2 workspace exists
mkdir -p ~/ros2/src_con/ext
cd ~/ros2/src_con

# Adding support for ros2_control
if [ "$FORCE_INSTALL" = true ] || [ ! -d /ext/ros_controls ]; then
    echo "Setting up ros2_controller in src_con/ext..." >> /tmp/entrypoint.log
    vcs import --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.$ROS_DISTRO.repos ext
    rosdep update --rosdistro=$ROS_DISTRO
    sudo apt-get update
    rosdep install --from-paths ext --ignore-src -r -y
    . /opt/ros/${ROS_DISTRO}/setup.sh
    colcon build --symlink-install
else
    echo "ros2_controller already exists in src_con. Skipping import and src_con." >> /tmp/entrypoint.log
fi

# Debug: Log the directory structure of ros2
ls -l ~/ros2 >> /tmp/entrypoint.log

# Source the ROS 2 setup
. ~/ros2/src_con/install/setup.bash

# Come back to ros2
cd ~/ros2/

# Execute the main container command
exec "$@"

