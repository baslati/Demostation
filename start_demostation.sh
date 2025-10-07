#!/bin/bash
set -e

# In Projektordner wechseln
cd /home/parallels/Demostation

echo "Setze X11 Rechte für Docker..."
xhost +local:docker

echo "Im Container bitte ausführen:"
echo "colcon build --symlink-install"
echo "ros2 launch custom_ur_moveit_config combined_ur3e.launch.py use_fake_hardware:=true"

echo "Starte Docker Container..."
docker run --name demostation-ur3 -it --rm --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --env="MESA_GL_VERSION_OVERRIDE=3.3" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$PWD/ros2_ur3_project/workspace/custom_code:/workspace/custom_code" \
  --privileged \
  ur3-ros2
