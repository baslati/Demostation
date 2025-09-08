#!/bin/bash
set -e

# ROS 2 sourcen
source /opt/ros/humble/setup.bash

# Workspace vorbereiten
cd /workspace

# Umgebung laden
source install/setup.bash

# Übergabe an Bash oder anderes Kommando
exec "$@"

