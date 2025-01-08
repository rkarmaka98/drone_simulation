# Use the official ROS 2 Jazzy base image
FROM osrf/ros:jazzy-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=jazzy\
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility

# Install prerequisites and dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    build-essential \
    git \
    wget \
    gnupg2 \
    lsb-release \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    libx11-xcb1 \
    libxcb1 \
    libxcb-util1 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-shm0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    x11-apps \
    libgl1 \
    libgl1-mesa-dri \
    libx11-6 \
    libglx-mesa0 \
    libgles2 \
    libglvnd-dev \
    libgl1-mesa-dev \
    mesa-utils \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository and clean up duplicates
RUN rm -f /etc/apt/sources.list.d/ros2*.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install Gazebo Harmonic and ROS-Gazebo integration packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 controllers and related packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-hardware-interface \
    && rm -rf /var/lib/apt/lists/*

# Add NVIDIA support
RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && apt-get update \
    && apt-get install -y nvidia-container-toolkit \
    && rm -rf /var/lib/apt/lists/*


# Adding rqt support to ros2
RUN apt update && apt install -y \
'~nros-jazzy-rqt*'

# Adding turtlesim
RUN apt install ros-jazzy-turtlesim

# Installing colcon build package
RUN apt install python3-colcon-common-extensions

# Install mavros for gcs simulation with with flight controller
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras

# Install QGroundControl
RUN apt-get remove modemmanager -y \
    && apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y \
    && apt install libfuse2 -y \
    && apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

# Install FUSE
RUN apt-get update && apt-get install -y \
fuse \
&& rm -rf /var/lib/apt/lists/*

# Create a non-root user with a home directory
RUN useradd -m -s /bin/bash qgcuser && \
    echo "qgcuser:qgcpassword" | chpasswd && \
    usermod -a -G dialout qgcuser && \ 
    mkdir -p /home/qgcuser && \
    chown -R qgcuser:qgcuser /home/qgcuser

# Ensure the runtime directory exists with the correct permissions
RUN mkdir -p /tmp/runtime-docker && \
    chown -R qgcuser:qgcuser /tmp/runtime-docker

# Install sudo and configure it to not require a password for qgcuser
RUN apt-get update && apt-get install -y sudo && \
    echo "qgcuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add entrypoint script to handle runtime directory permissions
COPY entrypoint.sh /home/qgcuser/entrypoint.sh
RUN chmod +x /home/qgcuser/entrypoint.sh

# Copy the PX4 Autopilot source code from the host to the container
COPY ./lib/PX4-Autopilot /home/qgcuser/PX4-Autopilot

# Grant ownership of the PX4 directory to the non-root user
RUN chown -R qgcuser:qgcuser /home/qgcuser/PX4-Autopilot

# Copy scripts to container
COPY ./scripts/px4_sitl_gazebo_setup.sh /home/qgcuser/
RUN chown -R qgcuser:qgcuser /home/qgcuser/px4_sitl_gazebo_setup.sh
COPY ./scripts/qgroundcontrol_run.sh /home/qgcuser/
RUN chown -R qgcuser:qgcuser /home/qgcuser/qgroundcontrol_run.sh
COPY ./scripts/QGroundControl.AppImage /home/qgcuser/
RUN chown -R qgcuser:qgcuser /home/qgcuser/QGroundControl.AppImage
COPY ./scripts/px4_gz_run.sh /home/qgcuser/
RUN chown -R qgcuser:qgcuser /home/qgcuser/px4_gz_run.sh

# Set the entrypoint script
ENTRYPOINT ["/home/qgcuser/entrypoint.sh"]

# Switch to non-root user
USER qgcuser

# Set working directory for the user
WORKDIR /home/qgcuser

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Expose Gazebo default port
EXPOSE 11345
