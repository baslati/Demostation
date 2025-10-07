#!/bin/bash
set -e

# In Projektordner wechseln
cd /home/orin/Desktop/Demostation

echo "Setze X11 Rechte für Docker..."
xhost +local:docker

echo "Im Container bitte ausführen:"
echo "python3 /workspace/custom_code/cube_manipulator.py"

echo "Starte Docker Container..."
docker exec -it demostation-ur3 bash 
