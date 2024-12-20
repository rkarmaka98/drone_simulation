#!/bin/bash

# Set DISPLAY env variable value
export DISPLAY=:1
xhost +local:
xhost +SI:localuser:$(whoami)

# Remove all containers
if [ -z "$(docker ps -q)" ]; then
  echo "No running containers found."
else
  docker stop $(docker ps -q)
fi
docker rm $(docker ps -aq)
echo "Stopping and removing all containeers"

# Start the container with ros and gazebo	
echo "Creating new container with ros and gazebo"
docker run -it --gpus all     --env=DISPLAY=$DISPLAY     --env=QT_X11_NO_MITSHM=1     --env=XDG_RUNTIME_DIR=/tmp/runtime-docker     --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw     --volume=/tmp/runtime-docker:/tmp/runtime-docker:rw     --volume=/home/tony/workspace_ros/src:/root/ros2     --runtime=nvidia     --name ros2_gazebo_env     ros2-gazebo:jazzy-harmonic     bash

