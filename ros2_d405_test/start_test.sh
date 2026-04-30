#!/bin/bash
set -e

# Beide Container muessen dieselbe ROS Domain nutzen.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# In Projektordner wechseln
cd "$(dirname "$0")"

echo "Setze X11 Rechte für Docker..."
xhost +local:docker
echo "ROS_DOMAIN_ID wird verwendet: ${ROS_DOMAIN_ID}"

echo "Baue Docker Image (mit Host-Netzwerk für Internetzugriff)..."
docker build --network host -t d405-test-image .

echo "Starte Docker Container..."
echo "Zum Testen der Kamera:"
echo "realsense-viewer"
echo "ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true enable_sync:=true"
echo " docker exec -it d405_test_container bash
python3 /workspace/src/custom_packages/custom_code/template_matching_roi_icp_node_ondemand.py"
echo " docker exec -it d405_test_container bash
rviz2 -d /workspace/src/custom_packages/custom_code/config.rviz"
echo "python3 /workspace/src/custom_packages/custom_code/template_scan_grasp_point_d405.py"


# Check if 'docker compose' is available, otherwise try 'docker-compose'
if docker compose version >/dev/null 2>&1; then
    docker compose up -d
else
    docker-compose up -d
fi

echo "Container gestartet. Gebe Shell frei..."
docker exec -it d405_test_container bash

# Container nach dem Verlassen der Shell stoppen und löschen
echo "Stoppe und lösche Container..."
docker compose down
