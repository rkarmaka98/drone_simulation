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

# Execute the main container command
exec "$@"

