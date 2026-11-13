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

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace

CMD ["bash"]