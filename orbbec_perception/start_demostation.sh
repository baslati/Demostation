#!/bin/bash
set -e

# In Projektordner wechseln
cd /home/orin/Desktop/Demostation

echo "Setze X11 Rechte für Docker..."
xhost +local:docker


echo "Im Container bitte ausführen:"
echo "colcon build --symlink-install"
echo "ros2 launch <dein_perception_package> <dein_launchfile>.py"

echo "Starte Docker Container..."
docker run --name demostation-orbbec -it --rm --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --env="MESA_GL_VERSION_OVERRIDE=3.3" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$PWD/workspace/custom_code:/workspace/custom_code" \
  --privileged \
  orbbec-perception
