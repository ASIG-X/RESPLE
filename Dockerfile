# ROS base
FROM osrf/ros:humble-desktop-full

# Choose bash shell
SHELL ["/bin/bash", "-c"]

# Environment Variables
ENV HOME=/root

# Update apt
RUN apt update

# Install git and SSH client
RUN apt install -y git openssh-client

# Add public github and bitbucket keys
RUN mkdir -p -m 0600 $HOME/.ssh
RUN ssh-keyscan github.com >> $HOME/.ssh/known_hosts
RUN ssh-keyscan bitbucket.org >> $HOME/.ssh/known_hosts

# Install system dependencies
RUN apt update && apt install -y \
    libeigen3-dev \
    libomp-dev \
    libpcl-dev \
    ros-humble-pcl*

# Install mcap support
RUN apt install -y ros-humble-rosbag2-storage-mcap

# Install gdb support
RUN apt install -y gdb gdbserver

# Add colcon bash completion
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> $HOME/.bashrc

# Create ROS2 Workspace
RUN mkdir -p $HOME/ros2_ws/src

# Build workspace packages, mounting only for the build
WORKDIR $HOME/ros2_ws
RUN --mount=type=bind,destination=$HOME/ros2_ws/src/RESPLE source /opt/ros/humble/setup.bash && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to resple

# Add workspace to default source
RUN echo "source $HOME/ros2_ws/install/setup.bash" >> $HOME/.bashrc

