#!/bin/bash

# Debug: Log the input arguments
echo "Running entrypoint.sh with arguments: $@" > /tmp/entrypoint.log

# Ensure the runtime directory exists with the correct permissions
mkdir -p /tmp/runtime-docker
chmod 0700 /tmp/runtime-docker

# Debug: Check if runtime directory exists and permissions
ls -ld /tmp/runtime-docker >> /tmp/entrypoint.log

# Adding local model path to the Gazebo sim
export GZ_SIM_RESOURCE_PATH=/root/local_models/

# Sourcing ROS2 and adding it to .bashrc
touch ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

# Define the QGroundControl filename
QGC_FILE="QGroundControl.AppImage"

# Check if QGroundControl already exists
if [ -f "$QGC_FILE" ]; then
    echo "QGroundControl already exists. Skipping download."
else
    echo "QGroundControl not found. Downloading..."
    wget -O "$QGC_FILE" https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
    chmod +x "$QGC_FILE"
    echo "QGroundControl downloaded and made executable."
fi

# Setting bootup directory as ros2
cd /home/qgcuser

# Execute the main container command
exec "$@"