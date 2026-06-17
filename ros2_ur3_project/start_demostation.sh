#!/bin/bash
set -e

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
cd "$(dirname "$0")/.."

ROS_SETUP="source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash"

echo "========================================"
echo "  Demostation UR3 Autostart"
echo "========================================"

echo "[1/5] Setze X11 Rechte..."
xhost +local:docker

echo "[2/5] Baue Docker Image..."
docker build --network host -t ur3-ros2 -f ros2_ur3_project/Dockerfile ros2_ur3_project

echo "[3/5] Starte Container..."
docker rm -f demostation-ur3 >/dev/null 2>&1 || true
docker run --name demostation-ur3 -d --rm --net=host \
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
  ur3-ros2 \
  sleep infinity

echo "[4/5] Baue ROS Workspace..."
docker exec demostation-ur3 bash -c \
  "source /opt/ros/humble/setup.bash && \
   cd /workspace && colcon build --symlink-install \
   > /tmp/colcon.log 2>&1"
echo "    Build fertig."

echo "[5/5] Starte ROS Launch + RViz + Nodes..."
docker exec -d demostation-ur3 bash -c \
  "$ROS_SETUP && ros2 launch custom_ur_moveit_config combined_ur3e.launch.py \
     use_fake_hardware:=false \
     > /tmp/launch.log 2>&1"

echo "    Warte auf ROS Init (20s)..."
sleep 20

docker exec -d demostation-ur3 bash -c \
  "$ROS_SETUP && python3 /workspace/custom_code/ur3_grip_and_place.py \
     > /tmp/grip_and_place.log 2>&1"

docker exec -d demostation-ur3 bash -c \
  "$ROS_SETUP && python3 /workspace/custom_code/ur3_provide_from_storage.py \
     > /tmp/provide_from_storage.log 2>&1"

echo ""
echo "========================================"
echo "  Alles gestartet. Shell freigegeben."
echo "  Logs: /tmp/launch.log /tmp/grip_and_place.log"
echo "========================================"
docker exec -it demostation-ur3 bash

# Aufräumen beim Beenden
echo "Stoppe Container..."
docker rm -f demostation-ur3 >/dev/null 2>&1 || true
