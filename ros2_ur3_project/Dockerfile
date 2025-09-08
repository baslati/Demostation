FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Systempakete + ROS 2 Quellen einrichten ---
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

# Installiere ROS-Komponenten & UR-Pakete
RUN apt update && apt install -y \
    ros-humble-ur-robot-driver \
    ros-humble-ur-client-library \
    ros-humble-ur-moveit-config \
    ros-humble-ur-description \
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
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libegl1 \
    libgles2-mesa-dev mesa-utils \
    iputils-ping \
    ocl-icd-libopencl1 \
    opencl-headers \
    clinfo \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    iproute2 \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

WORKDIR /workspace

COPY workspace/src/ /workspace/src/


COPY workspace/custom_code/custom_ur_description 	/workspace/custom_code/custom_ur_description
COPY workspace/custom_code/custom_ur_moveit_config    	/workspace/custom_code/custom_ur_moveit_config

RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --rosdistro humble --os=debian:bookworm --from-paths src --ignore-src -r -y \
    --skip-keys="ament_clang_format ament_clang_tidy ros2_controllers_test_nodes launch_testing_ament_cmake launch_testing_ros hardware_interface_testing socat backward_ros"

RUN source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install

# optional: sicherstellen, dass pip aktuell ist
RUN python3 -m pip install --no-cache-dir --upgrade pip

# explizit PyPI nutzen (überschreibt evtl. gesetzten PIP_INDEX_URL)
RUN pip install --no-cache-dir -i https://pypi.org/simple \
    "fastapi==0.112.*" "uvicorn[standard]==0.30.*"

# Automatisch sourcen bei Shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
