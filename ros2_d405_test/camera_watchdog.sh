#!/bin/bash
# Ueberwacht den realsense2_camera-Prozess im laufenden Container und startet ihn
# bei einem Absturz (z.B. USB-Aussetzer "No such device") automatisch neu.
# Wird per docker exec -d gestartet, laeuft dauerhaft im Hintergrund weiter,
# solange der Container lebt (unabhaengig von restart: unless-stopped, das nur
# den Container selbst betrifft, nicht Prozesse darin).

set -u

ROS_SETUP="source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash"
LAUNCH_CMD="ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=848x480x5 \
  depth_module.color_profile:=848x480x5 \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  decimation_filter.enable:=true \
  spatial_filter.enable:=false \
  temporal_filter.enable:=true \
  hole_filling_filter.enable:=false"

CHECK_INTERVAL_SEC=3

while true; do
    if ! pgrep -f "realsense2_camera" >/dev/null 2>&1; then
        echo "[WATCHDOG] $(date '+%F %T') Kamera-Prozess nicht gefunden, starte neu..." >> /tmp/camera.log
        bash -c "$ROS_SETUP && $LAUNCH_CMD" >> /tmp/camera.log 2>&1 &
        sleep 8   # Kamera-Init abwarten, bevor der naechste Check greift
    fi
    sleep "$CHECK_INTERVAL_SEC"
done
