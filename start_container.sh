#!/bin/bash

# Set DISPLAY env variable value
export DISPLAY=:1
xhost +local:
xhost +SI:localuser:$(whoami)

CONTAINER_NAME="ros2_gazebo_env"
IMAGE_NAME="ros2-gazebo:jazzy-harmonic"

# Function to remove all containers
cleanup_containers() {
    if [ -z "$(docker ps -q)" ]; then
        echo "No running containers found."
    else
        docker stop $(docker ps -q)
    fi
    docker rm $(docker ps -aq)
    echo "Stopping and removing all containers"
}

# Check if the container is running
if docker ps --filter "name=$CONTAINER_NAME" --format '{{.Names}}' | grep -w "$CONTAINER_NAME" > /dev/null; then
    echo "Container $CONTAINER_NAME is already running. Executing bash inside it..."
    docker exec -it $CONTAINER_NAME bash
elif docker ps -a --filter "name=$CONTAINER_NAME" --format '{{.Names}}' | grep -w "$CONTAINER_NAME" > /dev/null; then
    # If the container exists but is stopped
    echo "Container $CONTAINER_NAME exists but is not running. Starting it..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    # Cleanup old containers and create a new one
    echo "No existing container found. Cleaning up old containers..."
    cleanup_containers

    # Create a new container
    echo "Creating new container with ros and gazebo"
    docker run -it --gpus all \
        --env=DISPLAY=$DISPLAY \
        --env=QT_X11_NO_MITSHM=1 \
        --env=XDG_RUNTIME_DIR=/tmp/runtime-docker \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume=/tmp/runtime-docker:/tmp/runtime-docker:rw \
        --volume=/home/tony/workspace_ros/src:/root/ros2 \
        --runtime=nvidia \
        --name $CONTAINER_NAME \
        $IMAGE_NAME \
        bash
fi