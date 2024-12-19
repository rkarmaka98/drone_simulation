# Docker build
docker build -t ros2-gazebo:jazzy-harmonic .


# Docker run command
docker run -it --gpus all     --env=DISPLAY=$DISPLAY     --env=QT_X11_NO_MITSHM=1     --env=XDG_RUNTIME_DIR=/tmp/runtime-docker     --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw     --volume=/tmp/runtime-docker:/tmp/runtime-docker:rw     --volume=/home/tony/workspace_ros:/root/ros2     --runtime=nvidia     --name ros2_gazebo_env     ros2-gazebo:jazzy-harmonic     bash
