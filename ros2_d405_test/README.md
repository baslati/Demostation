# ROS 2 Testumgebung fuer Realsense D405

Dieses Verzeichnis enthaelt eine Docker-Umgebung, um die Realsense D405 Kamera auf dem Jetson zu testen.

## Inhalt

- Dockerfile: Baut `librealsense` (mit CUDA) und `realsense-ros`.
- docker-compose.yml: Konfiguriert den Container mit USB-Zugriff und X11-Forwarding.
- workspace/src/custom_code/: Eigener Python-Code.

## Schnellstart

1. Container starten:

```bash
./start_test.sh
```

2. In einem Terminal im Container die Kamera starten:

docker exec -it d405_test_container bash

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true enable_sync:=true
```

3. In weiteren Container-Terminals die einzelnen Programme starten.

## Programm 1: aruco_pointcloud_cropper_snapshot

Datei: `workspace/src/custom_code/aruco_pointcloud_cropper_snapshot.py`

Zweck:
- ArUco-Marker erkennen
- Punktwolke relativ zum Marker interaktiv cropen
- Den akzeptierten Crop als Snapshot speichern (`.pcd`, Fallback `.npy`)

Start:

docker exec -it d405_test_container bash

```bash
cd /workspace/src/custom_code
python3 aruco_pointcloud_cropper_snapshot.py
```

Bedienung im Terminal:
- In Schritt 1 `Enter`: Snapshot aufnehmen
- In Schritt 2 Grenzen eingeben: `x_min x_max y_min y_max z_min z_max`
- In Schritt 3 `j` bestaetigt den Crop und startet Speichern
- Dateinamen eingeben oder mit `Enter` Standardnamen uebernehmen
- In Schritt 4 `q` beendet den Node

Relevante Topics:
- Input: `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`, `/camera/camera/depth/color/points`
- Output: `/cloud_full`, `/cloud_cropped`, `/snapshot_axes`

## Programm 2: preprocess_template_ransac_y_axis

Datei: `workspace/src/custom_code/preprocess_template_ransac_y_axis.py`

Zweck:
- Rohe Scan-Punktwolke bereinigen
- Tisch per RANSAC entfernen
- Groessten Objektcluster extrahieren
- Outlier entfernen
- Optional downsamplen/Normalen berechnen
- Auf Centroid zentrieren und entlang Y-Achse ausrichten

Start (Beispiel):

```bash
cd /workspace/src/custom_code
python3 preprocess_template_ransac_y_axis.py scans/cropv1.pcd --preview
```

Nutzliche Optionen:
- `--ransac-threshold 0.004`
- `--cluster-eps 0.010`
- `--cluster-min-points 100`
- `--no-downsample`
- `--normals`
- `-o templates/cropv1_clean_direction.pcd`

Ergebnis:
- Standard-Ausgabe: `templates/<name>_clean_direction.pcd`

## Programm 3: template_matching_roi_icp_node_ondemand

Datei: `workspace/src/custom_code/template_matching_roi_icp_node_ondemand.py`

Zweck:
- ArUco erkennen
- Nach Enter einmalige Tool-Erkennung (ROI + ICP)
- Tool-Pose publizieren und Status melden

Start:

docker exec -it d405_test_container bash

```bash
cd /workspace/src/custom_code
python3 template_matching_roi_icp_node_ondemand.py
```

Ablauf:
1. Node startet und wartet auf stabile ArUco-Erkennung (`aruco_0`).
2. Nach Aufforderung `Enter` druecken.
3. Einmaliger Scan wird ausgefuehrt.
4. Ergebnis wird publiziert.

Relevante Topics:
- Output Pose: `/tool_target_pose`
- Output Status: `/tool_detection_status`

## Empfohlener Gesamtworkflow

1. Kamera starten.
2. Mit `aruco_pointcloud_cropper_snapshot.py` gute Objekt-Scans erstellen.
3. Mit `preprocess_template_ransac_y_axis.py` daraus saubere Templates erzeugen.
4. Mit `template_matching_roi_icp_node_ondemand.py` die einmalige Detektion und Pose-Publikation ausfuehren.
