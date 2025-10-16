#!/usr/bin/env bash
set -euo pipefail

# ROS sourcen
source /opt/ros/humble/setup.bash
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

if [[ "${ORBBEC_AUTO_LAUNCH:-false}" == "true" ]]; then
  echo "[entrypoint] Launching Orbbec Femto Mega..."
  exec ros2 launch orbbec_camera femto_mega.launch.py \
    enable_color:=true enable_depth:=true enable_point_cloud:=true publish_tf:=true
else
  exec "$@"
fi
