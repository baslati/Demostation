# Minimal ROS 2 Humble + RealSense D415 + RViz helper image
# NOTE: This reuses the same Jetson/L4T base as your existing D415 project.
FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Fix ROS2 apt repo key (prevents: EXPKEYSIG F42ED6FBAB17C654)
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release ca-certificates \
        && rm -rf /var/lib/apt/lists/* && \
        apt-key del F42ED6FBAB17C654 || true && \
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg && \
        echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
            > /etc/apt/sources.list.d/ros2.list

# Pin: prevent Ubuntu OpenCV from being installed
RUN printf 'Package: libopencv*\nPin: release *\nPin-Priority: -1\n' > /etc/apt/preferences.d/block-ubuntu-opencv

# Core tooling + RViz + launch
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-joint-state-publisher \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Workspace skeleton
WORKDIR /workspace
RUN mkdir -p /workspace/src && \
    git clone --branch humble https://github.com/ros-perception/vision_opencv.git /workspace/src/vision_opencv && \
    git clone --branch ros2-development https://github.com/IntelRealSense/realsense-ros.git /workspace/src/realsense-ros

# Librealsense (SDK) build
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake build-essential pkg-config \
    libssl-dev libusb-1.0-0-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true && \
    make -j$(nproc) && make install && \
    ldconfig && \
    rm -rf /tmp/librealsense

# Dependencies for realsense2_camera
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-statistics-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-ament-cmake-cppcheck \
    ros-humble-ament-cmake-cpplint \
    ros-humble-ament-cmake-uncrustify \
    ros-humble-ament-cmake-flake8 \
    ros-humble-ament-cmake-pep257 \
    ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# Rosdep (best effort in Docker builds)
RUN bash -lc 'source /opt/ros/humble/setup.bash && rosdep init || true' && \
    bash -lc 'source /opt/ros/humble/setup.bash && rosdep update || true'

# Build the workspace (realsense2_camera)
RUN bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace && colcon build --cmake-args -DAMENT_LINT_AUTO_SKIP=ON --symlink-install'

# RealSense udev rules (best effort)
RUN bash -lc 'cd /workspace/src/realsense-ros/scripts && bash patch-udev-ros2.sh || true'

# Convenience: source in interactive shells
RUN echo "source /opt/ros/humble/install/setup.bash" >> /root/.bashrc && \
    echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
