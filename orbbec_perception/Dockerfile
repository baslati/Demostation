FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Basis + ROS 2 Quellen ---
RUN apt update && apt install -y \
    curl gnupg2 lsb-release sudo locales python3-pip git nano x11-apps gpg && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-key del F42ED6FBAB17C654 || true && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt update

# --- ROS + Build Tools + UR Abhängigkeiten ---
RUN apt update && apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros2launch \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    liburdfdom-tools \
    ros-humble-ur-msgs \
    ros-humble-ros2cli \
    ros-humble-ros2topic \
    ros-humble-warehouse-ros-sqlite \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    x11-apps \
    ros-humble-rosidl-default-generators \
    libgl1-mesa-glx libgl1-mesa-dri libegl1 libgles2-mesa-dev mesa-utils \
    iputils-ping iproute2 net-tools \
    ocl-icd-libopencl1 opencl-headers clinfo \
    build-essential cmake git \
    python3-colcon-common-extensions python3-flake8 python3-rosdep \
    python3-setuptools python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

WORKDIR /workspace

# --- Pin: verhindere Ubuntu-OpenCV (wir nutzen Jetsons OpenCV) ---
RUN printf 'Package: libopencv*\nPin: release *\nPin-Priority: -1\n' > /etc/apt/preferences.d/block-ubuntu-opencv

# --- Repositories klonen ---
RUN mkdir -p /workspace/src && \
    if [ ! -d /workspace/src/OrbbecSDK_ROS2 ]; then \
        git clone https://github.com/orbbec/OrbbecSDK_ROS2.git /workspace/src/OrbbecSDK_ROS2 && \
        cd /workspace/src/OrbbecSDK_ROS2 && git checkout v2-main; \
    fi

RUN mkdir -p /workspace/src && \
    if [ ! -d /workspace/src/vision_opencv ]; then \
        git clone --branch humble https://github.com/ros-perception/vision_opencv.git /workspace/src/vision_opencv; \
    fi

RUN mkdir -p /workspace/src && \
    if [ ! -d /workspace/src/image_transport_plugins ]; then \
        git clone --branch humble https://github.com/ros-perception/image_transport_plugins.git /workspace/src/image_transport_plugins; \
    fi

RUN mkdir -p /workspace/src && \
    if [ ! -d /workspace/src/image_pipeline ]; then \
        git clone --branch humble https://github.com/ros-perception/image_pipeline.git /workspace/src/image_pipeline; \
    fi

# --- deine eigenen Pakete hinzufügen (optional) ---
COPY workspace/src/ /workspace/src/

# --- minimale ROS-Dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-statistics-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-stereo-msgs \
    ros-humble-vision-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-ros \
    libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
 && rm -rf /var/lib/apt/lists/*

# --- manuell fehlende Systembibliotheken (Universe) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common && \
    add-apt-repository universe && \
    apt-get update && apt-get install -y \
    libogg-dev libtheora-dev && \
    rm -rf /var/lib/apt/lists/*

# --- rosdep: Ubuntu Jammy, OpenCV & nicht verfügbare Keys überspringen ---
RUN bash -lc 'source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --rosdistro humble --os=ubuntu:jammy \
      --from-paths src --ignore-src -r -y \
      --skip-keys="opencv opencv-dev cv-bridge image-publisher \
      ament_cmake_auto launch_testing launch_testing_ament_cmake ros_testing \
      ros-humble-image-publisher libopencv-dev python3-opencv \
      libopencv-imgproc-dev libopencv-core-dev libopencv-highgui-dev \
      libopencv-videoio-dev libopencv-calib3d-dev libopencv-features2d-dev \
      libopencv-objdetect-dev libopencv-dnn-dev libopencv-photo-dev \
      libopencv-shape-dev libopencv-stitching-dev libopencv-video-dev \
      libopencv-videostab-dev libopencv-contrib-dev"'

# --- ament_cmake_auto nachinstallieren (fehlt in ros-base Images) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ament-cmake-auto && \
    rm -rf /var/lib/apt/lists/*



# --- Build ---
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# --- Python + API Tools ---
RUN python3 -m pip install --no-cache-dir --upgrade pip --index-url https://pypi.org/simple && \
    pip install --no-cache-dir -i https://pypi.org/simple "fastapi==0.112.*" "uvicorn[standard]==0.30.*"

# --- Open3D separat installieren (wegen Abhängigkeitskonflikten) ---
#RUN pip install open3d

# --- Auto-Source bei Container-Start ---
RUN echo "unset AMENT_TRACE_SETUP_FILES" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

# --- Orbbec Udev-Rules (optional aber empfehlenswert) ---
RUN bash -lc 'if [ -f /workspace/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh ]; then \
      cd /workspace/src/OrbbecSDK_ROS2/orbbec_camera/scripts && \
      bash install_udev_rules.sh || true && \
      udevadm control --reload-rules || true && udevadm trigger || true; \
    fi'

# --- Entrypoint ---
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
