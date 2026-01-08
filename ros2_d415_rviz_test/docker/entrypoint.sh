#!/usr/bin/env bash
set -e

# Make GUI/RViz work (X11)
export QT_X11_NO_MITSHM=1

source /opt/ros/humble/install/setup.bash

# Source built workspace (realsense2_camera)
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

# Source optional user overlay
if [ -f /workspace/src/user_code/install/setup.bash ]; then
  source /workspace/src/user_code/install/setup.bash
fi

exec "$@"
