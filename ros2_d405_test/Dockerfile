FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 1) System und ROS Keys fixen (mit IPv4 gegen Timeouts)
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates sudo nano \
    && rm -rf /var/lib/apt/lists/* && \
    apt-key del F42ED6FBAB17C654 || true && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list

# 2) OpenCV Konflikte vermeiden (Jetson nutzt eigenes OpenCV)
RUN printf 'Package: libopencv*\nPin: release *\nPin-Priority: -1\n' > /etc/apt/preferences.d/block-ubuntu-opencv

# 3) Abhängigkeiten installieren
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-rosidl-default-generators \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    git \
    udev \
    cmake build-essential pkg-config \
    libssl-dev libusb-1.0-0-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# 4) Librealsense bauen (Notwendig für Jetson & D405 Support)
RUN git clone https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && \
    mkdir -p build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_EXAMPLES=true \
      -DBUILD_GRAPHICAL_EXAMPLES=true \
      -DFORCE_RSUSB_BACKEND=OFF \
      -DBUILD_WITH_CUDA=ON \
    && make -j$(nproc) && make install && \
    rm -rf /tmp/librealsense

# 4.1) Zusätzliche Build-Dependencies (nachträglich, um Cache zu nutzen)
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install -y --no-install-recommends \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-ament-cmake-cppcheck \
    ros-humble-ros-base \
    ros-humble-std-srvs \
    ros-humble-image-transport \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-vision-msgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 4.2) Python Libraries
RUN pip3 install --no-cache-dir ultralytics --index-url https://pypi.org/simple

# 5) ROSWorkspace einrichten
WORKDIR /workspace
RUN mkdir -p /workspace/src && \
    git clone --branch humble https://github.com/ros-perception/vision_opencv.git /workspace/src/vision_opencv && \
    git clone --branch ros2-development https://github.com/IntelRealSense/realsense-ros.git /workspace/src/realsense-ros

# 6) Workspace bauen
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 7) Environment Setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]