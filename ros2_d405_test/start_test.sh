#!/bin/bash
set -e

# Beide Container muessen dieselbe ROS Domain nutzen.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# In Projektordner wechseln
cd "$(dirname "$0")"

ROS_SETUP="source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash"

echo "Warte auf Systemstart und D405-Kamera (USB 8086:0b5b)..."
sleep 10
timeout=120
elapsed=0
while ! lsusb | grep -q "8086:0b5b"; do
    if [ $elapsed -ge $timeout ]; then
        echo "WARNUNG: D405 nach ${timeout}s nicht gefunden, starte trotzdem..."
        break
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
echo "    D405 erkannt (nach ${elapsed}s), warte 5s auf USB-Initialisierung..."
sleep 5

echo "========================================"
echo "  Demostation D405 Autostart"
echo "========================================"

echo "[1/6] Setze X11 Rechte..."
xhost +local:docker

echo "[2/6] Baue Docker Image (wird übersprungen wenn bereits vorhanden)..."
if ! docker image inspect d405-test-image >/dev/null 2>&1; then
    docker build --network host -t d405-test-image .
else
    echo "    Image d405-test-image bereits vorhanden, überspringe Build."
fi

echo "[3/6] Starte Container..."
if docker compose version >/dev/null 2>&1; then
    docker compose up -d
else
    docker-compose up -d
fi

echo "[4/6] Starte Kamera..."
docker exec -d d405_test_container bash -c \
  "$ROS_SETUP && ros2 launch realsense2_camera rs_launch.py \
     depth_module.depth_profile:=848x480x5 \
     depth_module.color_profile:=848x480x5 \
     pointcloud.enable:=true \
     align_depth.enable:=true \
     enable_sync:=true \
     decimation_filter.enable:=true \
     spatial_filter.enable:=false \
     temporal_filter.enable:=true \
     hole_filling_filter.enable:=false \
     > /tmp/camera.log 2>&1"

echo "    Warte auf Kamera-Init (8s)..."
sleep 8

echo "[5/6] Starte Erkennungs-Node + RViz..."
docker exec -d d405_test_container bash -c \
  "$ROS_SETUP && python3 /workspace/src/custom_packages/custom_code/template_matching_roi_icp_node_ondemand_avg.py \
     > /tmp/detection.log 2>&1"

docker exec -d d405_test_container bash -c \
  "$ROS_SETUP && rviz2 -d /workspace/src/custom_packages/custom_code/config.rviz \
     > /tmp/rviz.log 2>&1"

echo "[6/6] Starte Streamlit GUI + Chromium..."
docker exec -d d405_test_container bash -c \
  "$ROS_SETUP && streamlit run /workspace/cobot_gui.py \
     --server.headless true \
     --server.port 8501 \
     --server.address 0.0.0.0 \
     > /tmp/streamlit.log 2>&1"

sleep 4

flatpak run org.chromium.Chromium \
  --app=http://localhost:8501 \
  --window-position=0,900 \
  --window-size=1920,180 \
  --noerrdialogs \
  --disable-infobars \
  --no-first-run \
  --disable-session-crashed-bubble \
  --disable-gpu \
  --password-store=basic \
  --log-level=3 \
  2>/dev/null &
CHROMIUM_PID=$!

echo ""
echo "========================================"
echo "  Alles gestartet. Shell freigegeben."
echo "  Logs: /tmp/camera.log /tmp/detection.log"
echo "========================================"
docker exec -it d405_test_container bash

# Aufräumen beim Beenden
echo "Stoppe Container..."
docker compose down
pkill -f chromium 2>/dev/null || true
