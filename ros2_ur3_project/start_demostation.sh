#!/bin/bash
set -e

# Beide Container muessen dieselbe ROS Domain nutzen.
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"

# In Projektordner wechseln
cd /home/orin/Desktop/Demostation

echo "Setze X11 Rechte für Docker..."
xhost +local:docker

echo "ROS_DOMAIN_ID wird verwendet: ${ROS_DOMAIN_ID_VALUE}"

echo "Im Container bitte ausführen:"
echo "colcon build --symlink-install"
echo "ros2 launch custom_ur_moveit_config combined_ur3e.launch.py use_fake_hardware:=false"

echo "Baue UR3 Docker Image..."
docker build --network host -t ur3-ros2 -f ros2_ur3_project/Dockerfile ros2_ur3_project

echo "Entferne alten Containernamen (falls vorhanden)..."
docker rm -f demostation-ur3 >/dev/null 2>&1 || true

echo "Starte Docker Container..."
docker run --name demostation-ur3 -it --rm --net=host \
  --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}" \
  --env="FASTDDS_BUILTIN_TRANSPORTS=UDPv4" \
  --env="DISPLAY=$DISPLAY" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --env="MESA_GL_VERSION_OVERRIDE=3.3" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$PWD/ros2_ur3_project/workspace/custom_code:/workspace/custom_code" \
  --privileged \
  --runtime=nvidia \
  ur3-ros2
