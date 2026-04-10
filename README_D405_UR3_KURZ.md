# D405 + UR3e Kurz-Anleitung (aktueller Ablauf)

Diese Anleitung beschreibt den aktuell verwendeten Ablauf mit genau den Programmen,
die ihr heute genutzt habt:

- D405-Seite: template_matching_roi_icp_node_ondemand.py
- UR3-Seite: ur3_hover_grip_from_pose_node.py

Ziel:
- D405 erkennt Tool-Pose und published nach /tool_target_pose
- UR3-Node liest /tool_target_pose und fuehrt Hover + Grip + Home aus

## Live-Kurzstart (Copy-Paste)

Nur die noetigen Befehle fuer den Testbetrieb.

Terminal A (Host):

      export ROS_DOMAIN_ID=0
      cd /home/orin/Desktop/Demostation/ros2_d405_test
      ./start_test.sh

Terminal B (Host):

      export ROS_DOMAIN_ID=0
      docker exec -it d405_test_container bash

Terminal B (im Container):

      ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true enable_sync:=true

Terminal C (Host):

      export ROS_DOMAIN_ID=0
      docker exec -it d405_test_container bash

Terminal C (im Container):

      python3 /workspace/src/custom_packages/custom_code/template_matching_roi_icp_node_ondemand.py

Terminal D (Host):

      export ROS_DOMAIN_ID=0
      cd /home/orin/Desktop/Demostation/ros2_ur3_project
      ./start_demostation.sh

Terminal D (im Container):

      colcon build --symlink-install
      ros2 launch custom_ur_moveit_config combined_ur3e.launch.py use_fake_hardware:=false

Terminal E (Host):

      export ROS_DOMAIN_ID=0
      docker exec -it demostation-ur3 bash

Terminal E (im Container):

      python3 /workspace/custom_code/ur3_hover_grip_from_pose_node.py

Terminal F (optional Diagnose):

      docker exec -it d405_test_container bash
      ros2 topic echo /tool_target_pose

## 1) Voraussetzungen

- Docker laeuft
- X11 Zugriff erlaubt (GUI fuer RViz/Realsense Viewer)
- Beide Container verwenden dieselbe ROS Domain

Empfohlen vor dem Start in beiden Shells:

      export ROS_DOMAIN_ID=0

Wenn ihr eine andere Domain nutzt, dieselbe Zahl in beiden Projekten verwenden.

## 2) D405 Container starten

In Terminal A:

      cd /home/orin/Desktop/Demostation/ros2_d405_test
      ./start_test.sh

Hinweis:
- Das Skript baut das Image, startet den Container d405_test_container
   und oeffnet eine Shell im Container.

## 3) Kamera und D405 Erkennung starten

In Terminal B (Host):

      docker exec -it d405_test_container bash

Dann im Container:

      ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true enable_sync:=true

In Terminal C (Host):

      docker exec -it d405_test_container bash

Dann im Container:

      python3 /workspace/src/custom_packages/custom_code/template_matching_roi_icp_node_ondemand.py

Optional (Visualisierung) in Terminal D:

      docker exec -it d405_test_container bash
      rviz2 -d /workspace/src/custom_packages/custom_code/config.rviz

Wichtig:
- template_matching_roi_icp_node_ondemand.py published die Zielpose auf /tool_target_pose.

## 4) UR3 Container starten

In Terminal E:

      cd /home/orin/Desktop/Demostation/ros2_ur3_project
      ./start_demostation.sh

Hinweis:
- Das Skript startet den Container demostation-ur3 im Host-Netz.

## 5) MoveIt im UR3 Container starten

Im offenen Container-Terminal (demostation-ur3):

      colcon build --symlink-install
      ros2 launch custom_ur_moveit_config combined_ur3e.launch.py use_fake_hardware:=false

Warten bis Controller und MoveIt sauber oben sind.

## 6) Hover-Grip Node starten (aktuelles Hauptprogramm)

In Terminal F (Host):

      docker exec -it demostation-ur3 bash

Dann im Container:

      python3 /workspace/custom_code/ur3_hover_grip_from_pose_node.py

Damit ist der komplette Pfad aktiv:
- D405 Detection -> /tool_target_pose -> UR3 Hover/Grip Node

## 7) Schnelle Diagnose

In einem beliebigen ROS-Terminal:

      ros2 topic echo /tool_target_pose

Wenn dort keine Daten kommen:
- Pruefen, ob template_matching_roi_icp_node_ondemand.py laeuft
- Pruefen, ob Realsense Launch aktiv ist
- Pruefen, ob beide Container dieselbe ROS_DOMAIN_ID haben

## 8) Was wir heute gemacht haben (Zusammenfassung)

### A) ur3_hover_grip_from_pose_node.py verbessert

- Planungs- und Ausfuehrungsfluss an das funktionierende Referenzverhalten angenaehert
- Stabilere Ausfuehrung bei Service/Action Calls (Timeout- und Fehlerpfade bereinigt)
- Home-Pose auf Tischkoordinaten umgestellt und als eigenes, klares Schema hinterlegt
- Home-Orientierung mit demselben Orientierungsschema wie Zielpose umgesetzt

### B) Koordinatenlogik geklaert

- Unterscheidung zwischen Tischursprung und ArUco-Referenz sauber herausgearbeitet
- Erklaert, wann TABLE_ORIGIN_IN_BASE_* benutzt wird
- Erklaert, wann ARUCO_IN_BASE_* benutzt wird
- Inkonsistenzen durch gemischte Annahmen zu Referenzrahmen reduziert

### C) ArUco/Tool-Pose Referenz geprueft

- Nachvollzogen, in welchem Referenzpunkt die Tool-Pose aktuell publiziert wird
- Grundlage geschaffen, um bei Bedarf einen festen Offset (z. B. Marker-Ecke statt Marker-Zentrum) gezielt einzubauen

### D) Heute aufgetretene Fehler und wie sie korrigiert wurden

- Problem: Instabiles Verhalten durch Threading/Busy-Zustaende
   Korrektur: Busy-Handling und Ablaufreihenfolge im Node stabilisiert

- Problem: Verwechslung der Bezugssysteme (Tisch vs. ArUco)
   Korrektur: Klar getrennte Konstanten und erklaerte Umrechnungspfade

- Problem: Unklare Home-Definition
   Korrektur: Home explizit in Tischkoordinaten mit definierter Orientierung gesetzt

### E) Offene Punkte fuer den naechsten Test

- Falls Pose-Nullpunkt oben links statt Marker-Zentrum benoetigt wird:
   festen Offset in der D405 Pose-Pipeline eintragen und gegenmessen.
- Nach jedem groesseren Umbau einmal End-to-End pruefen:
   D405 Publish, Topic sichtbar, UR3 Empfang, Motion erfolgreich.