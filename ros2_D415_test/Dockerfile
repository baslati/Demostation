# Minimal ROS 2 Humble + RealSense D415 + RViz image for Seeed Jetson Orin
FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]


# Install core ROS2 tools, RViz, RealSense camera node, and lsusb
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-realsense2-camera \
    python3-colcon-common-extensions \
    python3-rosdep \
    usbutils \
    git \
    cmake build-essential pkg-config libssl-dev libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install RealSense SDK 2.0 from GitHub (RSUSB backend, keine Kernel-Patches nötig)
RUN git clone https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false && \
    make -j$(nproc) && make install && \
    ldconfig && \
    rm -rf /tmp/librealsense

# Optional: udev rules for RealSense (best effort)
RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0ad3", MODE="0666"' > /etc/udev/rules.d/99-realsense.rules || true

# Workspace skeleton
WORKDIR /workspace
RUN mkdir -p /workspace/src

# Convenience: source in interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
