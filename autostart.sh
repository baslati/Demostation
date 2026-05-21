#!/bin/bash
set -e

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
cd /home/orin/Desktop/Demostation

echo "========================================"
echo "  Demostation Autostart"
echo "========================================"

echo "[1/4] Setze X11 Rechte..."
xhost +local:docker

echo "[2/4] Baue Docker Image..."
docker build --network host -t ur3-ros2 -f ros2_ur3_project/Dockerfile ros2_ur3_project

echo "[3/4] Entferne alten Container (falls vorhanden)..."
docker rm -f demostation-ur3 >/dev/null 2>&1 || true

echo "[4/4] Starte Container mit automatischem Setup..."
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
  ur3-ros2 \
  bash -c "
    cleanup() {
      echo ''
      echo '--- Stoppe alle Prozesse (Strg+C erkannt) ---'
      [ -n \"\$PY_PID\" ] && kill \$PY_PID 2>/dev/null
      [ -n \"\$ROS_PID\" ] && kill \$ROS_PID 2>/dev/null
      exit 0
    }
    trap cleanup SIGINT SIGTERM

    echo '--- [1/3] Baue ROS Workspace ---'
    cd /workspace
    colcon build --symlink-install
    source /workspace/install/setup.bash

    echo '--- [2/3] Starte ROS Launch + RViz ---'
    ros2 launch custom_ur_moveit_config combined_ur3e.launch.py use_fake_hardware:=false &
    ROS_PID=\$!

    echo '--- Warte auf ROS Initialisierung (20s) ---'
    sleep 20

    echo '--- [3/3] Starte Steuerungsskript ---'
    echo '--- (Strg+C beendet alles sauber, oder: docker stop demostation-ur3) ---'
    python3 /workspace/custom_code/ur3_grip_and_place.py &
    PY_PID=\$!

    wait \$PY_PID
    cleanup
  "
