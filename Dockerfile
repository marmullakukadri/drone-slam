FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Tirane

RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    nano \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# SLAM stack + desktop helpers (added for the drone_slam package)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-tf2-tools \
    fluxbox \
    xterm \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace

CMD ["bash"]