# ROS 2 Testumgebung für Realsense D405

Dieses Verzeichnis enthält eine Docker-Umgebung, um die Realsense D405 Kamera auf dem Jetson zu testen. Setup ist angelehnt an das `ros2_ur3_project`.

## Struktur
- **Dockerfile**: Baut `librealsense` (mit CUDA) und `realsense-ros`.
- **docker-compose.yml**: Konfiguriert den Container mit USB-Zugriff und X11-Forwarding.
- **workspace/src**: Hier kann eigener Code abgelegt werden.

## Nutzung

1. **Starten**:
   ```bash
   ./start_test.sh
   ```
   Dies baut (falls nötig) den Container und öffnet eine Shell.

2. **Kamera testen (GUI)**:
   In der Container-Shell:
   ```bash
   realsense-viewer
   ```

3. **Kamera testen (ROS 2)**:
   In der Container-Shell:
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```
   In einem zweiten Terminal (via `docker exec -it d405_test_container bash`):
   ```bash
   rviz2
   ```

4. **Zangen-Erkennung mit YOLO starten**:
   Nach dem Starten des Containers und Sourcen der Umgebung:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch pliers_yolo_detection pliers_yolo.launch.py
   ```
   - **Video-Feed anzeigen**: In einem separaten Terminal (via `docker exec -it d405_test_container bash`):
     ```bash
     rqt
     ```
     Dann in rqt: Plugins > Visualization > Image View, und das Topic `/pliers/debug_image` auswählen.
   - **3D-Visualisierung**: In einem separaten Terminal:
     ```bash
     rviz2
     ```
     Füge einen Marker hinzu (Add > Marker) und wähle das Topic `/pliers/markers`.
   - **Anpassungen**: Parameter wie `yolo_model` (z.B. 'yolov8n.pt') oder `target_class_id` (76 für Scheren/Zangen) können im Launch-File geändert werden.
