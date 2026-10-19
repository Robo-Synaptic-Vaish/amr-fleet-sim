FROM osrf/ros:jazzy-desktop-full

# Install Gazebo Harmonic + ROS-Gazebo bridge
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-vision-msgs \
    python3-pip \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Auto-source ROS 2 in every new shell inside the container
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /root/ws

CMD ["bash"]