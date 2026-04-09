# D405 + UR3 Kurzanleitung

## ⚠️ WICHTIG: ROS_DOMAIN_ID

Beide Container **MÜSSEN die gleiche ROS_DOMAIN_ID haben**, sonst sehen sie sich nicht gegenseitig!

Die Startskripte setzen jetzt automatisch `ROS_DOMAIN_ID=0` (oder deinen bereits gesetzten Wert).

Überprüfe danach in BEIDEN Containern:
```bash
echo "ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
```

Falls unterschiedlich: Siehe [TROUBLESHOOTING_ROS_DOMAIN.md](TROUBLESHOOTING_ROS_DOMAIN.md)

---

## 1) Docker starten

Terminal A (D405):
cd /home/orin/Desktop/Demostation/ros2_d405_test
./start_test.sh

Terminal B (UR3):
cd /home/orin/Desktop/Demostation/ros2_ur3_project
./start_demostation.sh

## 2) Im D405 Container starten

In der D405-Container-Shell:
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true enable_sync:=true

In einer zweiten D405-Container-Shell:
docker exec -it d405_test_container bash
python3 /workspace/src/custom_packages/custom_code/d405_capture_to_pose_node.py

Hinweis:
- Enter startet genau eine Aufnahme.
- Bei Fehlschlag kommt: Keine Zange gefunden.
- Erfolgreiche Pose wird auf /tool_target_pose publiziert.

## 3) Im UR3 Container starten

In der UR3-Container-Shell:
cd /workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch custom_ur_moveit_config combined_ur3e.launch.py use_fake_hardware:=false

In einer zweiten UR3-Container-Shell:
docker exec -it demostation-ur3 bash
source /opt/ros/humble/setup.bash
python3 /workspace/custom_code/ur3_hover_grip_from_pose_node.py

Der UR3 Node gibt aus:
```
[INFO] ═══════════════════════════════════════════════════════
[INFO]   UR3 HOVER + GRIP FROM POSE NODE STARTEN
[INFO] ═══════════════════════════════════════════════════════
[INFO] [INIT] ROS_DOMAIN_ID=<WERT>
[INFO] ... ✓ NODE BEREIT - WARTE AUF ZIELPOSE
```

Falls das nicht ausgegeben wird → Siehe [TROUBLESHOOTING_ROS_DOMAIN.md](TROUBLESHOOTING_ROS_DOMAIN.md)

## 4) Bedienung

1. Stelle sicher, dass beide Nodes Logs ausgeben (siehe oben)
2. Kommunikationscheck in D405-Container:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic info /tool_target_pose
   ```
   Erwartet: `Publisher count >= 1` und `Subscription count >= 1`
3. Kommunikationscheck in UR3-Container:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic info /tool_target_pose
   ```
   Erwartet: `Publisher count >= 1` und `Subscription count >= 1`
4. Im D405 Terminal Enter druecken
5. Der UR3 Node sollte SOFORT Logs mit `[EMPFANGEN]` ausgeben
6. Bei Treffer macht UR3 automatisch:
   - 5 cm ueber Ziel fahren
   - Greifer schliessen
   - 2 Sekunden halten
   - Greifer oeffnen
   - zur Home Pose zurueckfahren

## 5) Wichtige Parameter

UR3 Parameter in:
- /home/orin/Desktop/Demostation/ros2_ur3_project/workspace/custom_code/ur3_hover_grip_from_pose_node.py

Besonders relevant:
- HOVER_Z_OFFSET_M (aktuell 0.05)
- HOLD_SECONDS (aktuell 2.0)
- HOME_X_M, HOME_Y_M, HOME_Z_M
- HOME_RX, HOME_RY, HOME_RZ
- ARUCO_IN_BASE_* (Kalibrierung ArUco -> base_link)

D405 Parameter in:
- /home/orin/Desktop/Demostation/ros2_d405_test/workspace/src/custom_code/d405_capture_to_pose_node.py

Besonders relevant:
- CAPTURE_TIMEOUT_SEC
- MARKER_FRAME
- DETECTED_FRAME
