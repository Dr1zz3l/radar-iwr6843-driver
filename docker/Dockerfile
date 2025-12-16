FROM osrf/ros:noetic-desktop-full

# Install useful dev tools
RUN apt-get update && apt-get install -y \
    usbutils \
    udev \
    git \
    build-essential \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace structure
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Initialize rosdep
RUN rosdep update

# Auto-source ROS + workspace
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /root/catkin_ws

CMD ["/bin/bash"]