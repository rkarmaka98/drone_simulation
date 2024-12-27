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

touch ~/.bashrc \
&& echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

source ~/.bashrc

# Setting bootup directory as ros2
cd /root/

# Execute the main container command
exec "$@"

