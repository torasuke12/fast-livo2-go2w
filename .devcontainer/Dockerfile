FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-default-generators \
    ros-humble-sophus \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch v1.3.1 https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j"$(nproc)" && \
    make install && \
    cd / && \
    rm -rf Livox-SDK2 && \
    ldconfig
