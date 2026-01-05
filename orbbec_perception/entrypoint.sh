#!/usr/bin/env bash
set -eo pipefail

# ROS environment is sourced in .bashrc

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

if [[ "${ORBBEC_AUTO_LAUNCH:-false}" == "true" ]]; then
  echo "[entrypoint] Launching Orbbec Femto Mega..."
  exec ros2 launch orbbec_camera femto_mega.launch.py \
    enable_color:=true enable_depth:=true enable_point_cloud:=true publish_tf:=true
else
  exec "$@"
fi
