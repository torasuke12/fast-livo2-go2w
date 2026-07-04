FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV FAST_LIVO2_WS=/workspace/fast_livo2_go2w/src/fast-livo2-go2w/humble_ws
ENV CYCLONEDDS_URI=file:///workspace/fast_livo2_go2w/src/fast-livo2-go2w/humble_ws/src/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
    pkg-config \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    udev \
    usbutils \
    wget \
    libboost-all-dev \
    libeigen3-dev \
    libglfw3-dev \
    libgtk-3-dev \
    libopencv-dev \
    libpcap-dev \
    libpcl-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    ros-humble-ament-cmake \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    ros-humble-diagnostic-updater \
    ros-humble-image-transport \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

WORKDIR /workspace/fast_livo2_go2w/src/fast-livo2-go2w/humble_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
