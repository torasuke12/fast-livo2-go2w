FROM ubuntu:jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV FAST_LIVO2_WS=/external/humble_ws
ENV CYCLONEDDS_URI=file:///external/humble_ws/src/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml

RUN set -eux; \
    codename="$(. /etc/os-release && echo "$VERSION_CODENAME")"; \
    arch="$(dpkg --print-architecture)"; \
    case "$arch" in \
      amd64|i386) \
        ubuntu_uri="http://archive.ubuntu.com/ubuntu"; \
        security_uri="http://security.ubuntu.com/ubuntu"; \
        ;; \
      *) \
        ubuntu_uri="http://ports.ubuntu.com/ubuntu-ports"; \
        security_uri="$ubuntu_uri"; \
        ;; \
    esac; \
    rm -f /etc/apt/sources.list /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; \
    { \
      echo "deb $ubuntu_uri $codename main restricted universe multiverse"; \
      echo "deb $ubuntu_uri $codename-updates main restricted universe multiverse"; \
      echo "deb $ubuntu_uri $codename-backports main restricted universe multiverse"; \
      echo "deb $security_uri $codename-security main restricted universe multiverse"; \
    } > /etc/apt/sources.list

RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
      apt-utils \
      bash-completion \
      ca-certificates \
      curl \
      gnupg2 \
      lsb-release \
      software-properties-common && \
    rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
      ros-humble-desktop \
      build-essential \
      cmake \
      git \
      iputils-ping \
      libboost-all-dev \
      libeigen3-dev \
      libglfw3-dev \
      libgtk-3-dev \
      libopencv-dev \
      libpcap-dev \
      libpcl-dev \
      libssl-dev \
      libudev-dev \
      libusb-1.0-0-dev \
      net-tools \
      pkg-config \
      python3-argcomplete \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      python3-vcstool \
      tmux \
      udev \
      usbutils \
      vim \
      wget \
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
      ros-humble-rosidl-default-generators \
      ros-humble-rosidl-generator-dds-idl \
      ros-humble-sophus \
      ros-humble-tf2-geometry-msgs \
      ros-humble-tf2-ros && \
    python3 -m pip install --no-cache-dir catmux distro transforms3d websocket-client && \
    rosdep init || true && \
    rm -rf /var/lib/apt/lists/*

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

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /external/humble_ws/install/setup.bash ] && source /external/humble_ws/install/setup.bash" >> /root/.bashrc && \
    echo "[ -f /external/humble_ws/src/unitree_ros2/setup.sh ] && source /external/humble_ws/src/unitree_ros2/setup.sh" >> /root/.bashrc

WORKDIR /external

CMD ["bash"]
