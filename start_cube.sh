#!/bin/bash
set -e

# In Projektordner wechseln
cd /home/parallels/Demostation

echo "Setze X11 Rechte für Docker..."
xhost +local:docker

echo "Im Container bitte ausführen:"
echo "cd /workspace/custom_code"
echo "python3 cube_manipulator.py"

echo "Starte Docker Container..."
docker exec -it demostation-ur3 bash 
