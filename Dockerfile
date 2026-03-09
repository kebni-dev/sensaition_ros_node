FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libboost-system-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["bash"]

# docker build -t kebni-driver .

# docker run -it --rm --device=/dev/ttyUSB0 -v "$(pwd)":/workspace kebni-driver

# docker run -it --rm -v "$(pwd)":/workspace kebni-driver

# colcon build && source install/setup.bash && ros2 launch kebni_driver kebni_driver.launch.py