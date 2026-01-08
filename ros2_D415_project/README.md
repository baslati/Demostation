Führe im Container diese Befehle aus, um nur unser Paket zu bauen und zu testen:

Nur unser Paket bauen (ohne symlink):
colcon build --packages-select tool_handling_perception
Umgebung sourcen:
source /opt/ros/humble/setup.bash
source install/setup.bash
Paket testen:
ros2 pkg list | grep tool_handling
RealSense starten:
ros2 run tool_handling_perception 02_realsense_launch.py &
Topics checken:
ros2 topic list
Camera Check starten:
ros2 run tool_handling_perception 01_camera_check.py
Wenn du die anderen Pakete brauchst (wie realsense2_camera), müssen wir das Problem mit camera_calibration lösen, aber für unsere Tests reicht unser Paket. Sag Bescheid, was passiert!

root@demojetson:/workspace# source /opt/ros/humble/setup.bash
source install/setup.bash
root@demojetson:/workspace# ros2 pkg list | grep tool_handling
tool_handling_perception
root@demojetson:/workspace# ros2 run tool_handling_perception 02_realsense_launch.py &
[1] 9973
root@demojetson:/workspace# No executable found
ros2 topic list             ros2 topic list
/parameter_events
/rosout
[1]+  Exit 1                  ros2 run tool_handling_perception 02_realsense_launch.py
root@demojetson:/workspace# 


Rolle: Du bist ein Senior Robotics Engineer und Experte für ROS 2 auf NVIDIA Jetson Hardware.

Kontext: Ich entwickle eine Masterarbeit ("Automatisierte Werkzeughandhabung") mit einem UR3e Roboter und einer Intel RealSense D415 (Eye-in-Hand am Greifer) auf einem Seeed Jetson Orin/Nano. Ich habe bereits funktionierende Konfigurationsdateien für Docker.

Deine Aufgabe: Richte mein ROS 2 Workspace-Skelett ein und erstelle die Python-Logik für die Bildverarbeitung.

1. Die gegebenen Dateien (Source of Truth - NICHT ändern):

A) Dockerfile: (Es installiert ROS 2 Humble, baut vision_opencv und realsense aus dem Source für Jetson/L4T Unterstützung). Gehe davon aus, dass dieses Dockerfile so existiert wie vom User bereitgestellt.

B) docker-compose.yml:

YAML
version: '3.8'
services:
  realsense-perception:
    build:
      context: ..
      dockerfile: docker/Dockerfile
    image: realsense-perception
    container_name: realsense_perception_demostation
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ${HOME}/.Xauthority:/root/.Xauthority:rw
      # WICHTIG: Mount-Point unten beachten!
      - ../colcon_ws/src:/workspace/src/my_pkg 
    privileged: true
    devices:
      - "/dev/bus/usb:/dev/bus/usb"
    tty: true
    stdin_open: true
2. Anweisungen zur Anpassung (WICHTIG):

Volume Mount Anpassung: Im originalen Compose-File wurde ../colcon_ws:/workspace gemountet. Das ist gefährlich, da es die im Dockerfile kompilierten System-Pakete (in /workspace/src und /workspace/install) überschreiben würde.

Deine Aufgabe: Bitte passe die Ordnerstruktur so an, dass mein lokaler Code in einen Unterordner gemountet wird (z.B. /workspace/src/user_code), damit die System-Dependencies erhalten bleiben.

3. Zu erstellende Python-Skripte (ROS 2 Nodes):

Erstelle ein ROS 2 Paket tool_handling_perception und darin folgende Nodes:

Skript 1: 01_camera_check.py

Subscribes /camera/color/image_raw.

Nutzt CvBridge (Achtung: Import muss zur Jetson-kompatiblen Version passen).

Zeigt das Live-Bild in einem OpenCV Fenster an.

Skript 2: 02_realsense_launch.py (Launchfile)

Startet den realsense2_camera_node (Parameter: enable_pointcloud=true, align_depth=true).

Startet RViz2 mit einer Standard-Config.

Skript 3: 03_yolo_detect.py

Lädt ultralytics YOLOv8 (nutze 'yolov8n.pt' als Dummy, lade es zur Laufzeit runter falls nicht vorhanden).

Macht Inference auf dem ROS-Image.

Zeichnet Bounding Boxes.

Publisht das Ergebnisbild auf /vision/debug_img.

Skript 4: 04_pose_estimation.py (Kern-Logik mit ArUco)

Problem: Ich habe keine externe Hand-Eye-Kalibrierung.

Lösung: Ich nutze einen ArUco-Marker auf dem Tisch als "Welt-Referenz".

Logik:

Erkenne ArUco Marker ID 0 im Bild (nutze cv2.aruco).

Bestimme Pose des Markers relativ zur Kamera (T 
cam
marker
​	
 ).

Erkenne Zange (YOLO Bounding Box Zentrum + Tiefe an dieser Stelle aus Depth-Image).

Bestimme Pose der Zange relativ zur Kamera (T 
cam
tool
​	
 ).

Berechne Pose der Zange relativ zum Marker: T 
marker
tool
​	
 =(T 
cam
marker
​	
 ) 
−1
 ∗T 
cam
tool
​	
 .

Publishe diese Pose (x,y,z,r,p,y) auf Topic /vision/tool_pose_relative_to_marker.

Ausgabe: Erstelle mir die Ordnerstruktur, die package.xml, setup.py und den Python-Code für die 4 Skripte. Achte auf sauberes Error-Handling (z.B. wenn kein Marker im Bild ist).