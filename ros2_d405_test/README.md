# ROS 2 Testumgebung für Realsense D405

Dieses Verzeichnis enthält eine Docker-Umgebung, um die Realsense D405 Kamera auf dem Jetson zu testen. Setup ist angelehnt an das `ros2_ur3_project`.

## Struktur
- **Dockerfile**: Baut `librealsense` (mit CUDA) und `realsense-ros`.
- **docker-compose.yml**: Konfiguriert den Container mit USB-Zugriff und X11-Forwarding.
- **workspace/src/custom_code/**: Eigener Code (gemountet, bleibt nach Container-Stopp erhalten).
  - `pointcloud_snapshot_node.py` – Speichert Punktwolken-Snapshots von der D405
  - `preprocess_template.py` – Bereinigt rohe Scans zu sauberen Templates

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

---

## Template Matching Workflow

### Schritt 1: Punktwolken-Snapshots aufnehmen

Das Snapshot-Script abonniert die Punktwolke der D405 (inkl. RGB-Farben) und speichert sie als `.pcd`-Datei.

**Terminal 1 – Kamera starten:**
```bash
docker exec -it d405_test_container bash
source /workspace/install/setup.bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

**Terminal 2 – Snapshot-Node starten:**
```bash
docker exec -it d405_test_container bash
python3 /workspace/src/custom_packages/custom_code/pointcloud_snapshot_node.py
```

**Bedienung:**
- ENTER drücken → Dateiname eingeben (z.B. `pliers_long`) → Scan wird gespeichert
- Für jede Zange einen Scan erstellen: `pliers_long`, `pliers_side`, `pliers_round` etc.
- Scans werden unter `/workspace/src/custom_packages/custom_code/scans/` gespeichert (Host: `./workspace/src/custom_code/scans/`)
- Ctrl+C zum Beenden

**Tipps für gute Scans:**
- Zange flach und isoliert auf den Tisch legen
- Kamera senkrecht von oben, ca. 20–25 cm Abstand
- Free-Drive des UR3e nutzen, um die D405 zu positionieren

### Schritt 2: Templates bereinigen (Preprocessing)

Die rohen Scans enthalten Tisch, Hintergrund und Rauschen. Das Preprocessing-Script entfernt diese und erstellt saubere Templates.

**Im Container:**
```bash
cd /workspace/src/custom_packages/custom_code
```

**Standard-Preprocessing mit Vorschau:**
```bash
python3 preprocess_template.py scans/pliers_long.pcd --preview
python3 preprocess_template.py scans/oben.pcd --preview
python3 preprocess_template.py scans/oben.pcd --z-min 0.18 --z-max 0.22 --x-range -0.10 0.10 --y-range -0.10 0.10 --preview
```

**Aggressiveres Cropping (z.B. Tisch bei 1cm, Zange bis 5cm Höhe):**
```bash
python3 preprocess_template.py scans/pliers_long.pcd --z-min 0.01 --z-max 0.05 --preview
```

**Mit Normalen für ICP-Matching:**
```bash
python3 preprocess_template.py scans/pliers_long.pcd --normals --preview
```

**Alle Parameter:**

| Parameter | Standard | Beschreibung |
|-----------|----------|--------------|
| `--z-min` | 0.005 | Min. Höhe in Metern (unter = Tisch) |
| `--z-max` | 0.10 | Max. Höhe in Metern |
| `--x-range` | -0.15 0.15 | X-Bereich in Metern |
| `--y-range` | -0.15 0.15 | Y-Bereich in Metern |
| `--voxel-size` | 0.001 | Voxelgröße (1mm) für Downsampling |
| `--no-downsample` | - | Kein Downsampling |
| `--outlier-neighbors` | 20 | Nachbarn für Outlier Removal |
| `--outlier-std` | 2.0 | Standardabweichungs-Schwellwert |
| `--preview` | - | Vorher/Nachher Visualisierung |
| `--normals` | - | Normalen berechnen (für ICP) |
| `-o` | auto | Eigener Ausgabepfad |

**Ausgabe:** Bereinigte Templates landen in `custom_code/templates/` (z.B. `pliers_long_clean.pcd`).

### Schritt 3: Alle Zangen scannen und bereinigen

```bash
# Scans aufnehmen (je Zangentyp einmal)
# → pliers_long.pcd, pliers_side.pcd, pliers_round.pcd

# Templates bereinigen
python3 preprocess_template.py scans/pliers_long.pcd --normals --preview
python3 preprocess_template.py scans/pliers_side.pcd --normals --preview
python3 preprocess_template.py scans/pliers_round.pcd --normals --preview
```

### Dateistruktur nach Scan & Preprocessing

```
workspace/src/custom_code/
├── pointcloud_snapshot_node.py    # Snapshot-Script
├── preprocess_template.py         # Preprocessing-Script
├── scans/                         # Rohe Scans
│   ├── pliers_long.pcd
│   ├── pliers_side.pcd
│   └── pliers_round.pcd
└── templates/                     # Bereinigte Templates
    ├── pliers_long_clean.pcd
    ├── pliers_side_clean.pcd
    └── pliers_round_clean.pcd
```

---

### Schritt 4: Template Matching & Pose-Erkennung

Das Template Matching Node erkennt Zangen in der Live-Punktwolke und publiziert deren Pose als TF-Frame.

**Pipeline:**
1. Live-Punktwolke empfangen (RealSense D405)
2. Tischebene per RANSAC automatisch entfernen
3. Voxel-Downsampling für Performance
4. Für jedes Template: Global Registration (FPFH) → Fine Registration (ICP)
5. Bestes Match auswählen (höchste Fitness)
6. Pose als TF-Frame publizieren → sichtbar in RViz2

**Terminal 1 – Kamera starten:**
```bash
docker exec -it d405_test_container bash
source /workspace/install/setup.bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

**Terminal 2 – Template Matching starten:**
```bash
docker exec -it d405_test_container bash
python3 /workspace/src/custom_packages/custom_code/template_matching_node.py
```

**Terminal 3 – Visualisierung in RViz2:**
```bash
docker exec -it d405_test_container bash
rviz2
```

In RViz2:
- **Fixed Frame** auf `camera_depth_optical_frame` setzen
- **Add → TF** hinzufügen → zeigt das Achsenkreuz der erkannten Zange (`detected_<template_name>`)
- **Add → PointCloud2** → Topic `/camera/camera/depth/color/points` → zeigt die Live-Punktwolke

**Ausgabe im Terminal:**
```
Match: pliers_long_clean | Fitness: 0.742 | RMSE: 0.0023m
Position: x=0.021, y=-0.015, z=0.215 m
```

**Konfiguration** (oben in `template_matching_node.py`):

| Parameter | Standard | Beschreibung |
|-----------|----------|--------------|
| `MATCH_INTERVAL` | 0.5 | Sekunden zwischen Matching-Versuchen |
| `MIN_FITNESS` | 0.3 | Mindest-Fitness damit ein Match akzeptiert wird (0-1) |
| `VOXEL_SIZE` | 0.002 | Voxelgröße für Downsampling (2mm) |
| `ICP_THRESHOLD` | 0.005 | Max Korrespondenz-Distanz für ICP (5mm) |
| `RANSAC_DISTANCE_THRESHOLD` | 0.005 | Toleranz für Tischebenen-Erkennung (5mm) |

**Hinweis:** Die Pose ist im Kamera-Koordinatensystem (`camera_depth_optical_frame`). Ohne Hand-Eye-Kalibrierung kann der Roboter diese Pose noch nicht direkt anfahren, aber die Erkennung kann isoliert getestet und evaluiert werden.

### Dateistruktur (komplett)

```
workspace/src/custom_code/
├── pointcloud_snapshot_node.py    # Snapshot-Script
├── preprocess_template.py         # Preprocessing-Script
├── template_matching_node.py      # Template Matching & TF-Publikation
├── scans/                         # Rohe Scans
│   ├── pliers_long.pcd
│   ├── pliers_side.pcd
│   └── pliers_round.pcd
└── templates/                     # Bereinigte Templates
    ├── pliers_long_clean.pcd
    ├── pliers_side_clean.pcd
    └── pliers_round_clean.pcd
```

python3 /workspace/src/custom_packages/custom_code/preprocess_template_ransac.py /workspace/src/custom_packages/custom_code/scans/cropv1.pcd \
  --ransac-threshold 0.005 \
  --cluster-eps 0.015 \
  --voxel-size 0.002 \
  --normals --preview


  [INFO] [1774883207.729002161] [rviz]: Message Filter dropping message: frame 'camera_depth_optical_frame' at time 1774883206.051 for reason 'discarding message because the queue is full'


  ros2 launch realsense2_camera rs_launch.py
pointcloud.enable:=true
align_depth.enable:=true
depth_module.depth_profile:=640x480x15
rgb_camera.color_profile:=640x480x15

root@demojetson:/workspace#  ros2 launch realsense2_camera rs_launch.py
pointcloud.enable:=true
align_depth.enable:=true
depth_module.depth_profile:=640x480x15
rgb_camera.color_profile:=640x480x15
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-03-30-15-11-17-887265-demojetson-12904
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: 🚀 Launching as Normal ROS Node
[INFO] [realsense2_camera_node-1]: process started with pid [12905]
[realsense2_camera_node-1] [INFO] [1774883478.054086193] [camera.camera]: RealSense ROS v4.57.0
[realsense2_camera_node-1] [INFO] [1774883478.054341467] [camera.camera]: Built with LibRealSense v2.57.6
[realsense2_camera_node-1] [INFO] [1774883478.054371965] [camera.camera]: Running with LibRealSense v2.57.6
[realsense2_camera_node-1]  30/03 15:11:18,063 ERROR [281472082569440] (context.cpp:41) No valid configuration file found at : /root/.realsense-config.json loading defaults
[realsense2_camera_node-1] [INFO] [1774883478.302915304] [camera.camera]: Device with serial number 409122274780 was found.
[realsense2_camera_node-1] 
[realsense2_camera_node-1] [INFO] [1774883478.303165010] [camera.camera]: Device with physical ID /sys/devices/platform/bus@0/3610000.usb/usb2/2-1/2-1.2/2-1.2:1.0/video4linux/video0 was found.
[realsense2_camera_node-1] [INFO] [1774883478.303202260] [camera.camera]: Device with name Intel RealSense D405 was found.
[realsense2_camera_node-1] [INFO] [1774883478.303497503] [camera.camera]: Device with port number 2-1.2 was found.
[realsense2_camera_node-1] [INFO] [1774883478.303531585] [camera.camera]: Device USB type: 3.2
[realsense2_camera_node-1] [INFO] [1774883478.303666150] [camera.camera]: getParameters...
[realsense2_camera_node-1] [INFO] [1774883478.304613612] [camera.camera]: JSON file is not provided
[realsense2_camera_node-1] [INFO] [1774883478.304674350] [camera.camera]: Device Name: Intel RealSense D405
[realsense2_camera_node-1] [INFO] [1774883478.304695279] [camera.camera]: Device Serial No: 409122274780
[realsense2_camera_node-1] [INFO] [1774883478.304712591] [camera.camera]: Device physical port: /sys/devices/platform/bus@0/3610000.usb/usb2/2-1/2-1.2/2-1.2:1.0/video4linux/video0
[realsense2_camera_node-1] [INFO] [1774883478.304730448] [camera.camera]: Device FW version: 5.15.1.55
[realsense2_camera_node-1] [INFO] [1774883478.304744273] [camera.camera]: Device Product ID: 0x0B5B
[realsense2_camera_node-1] [INFO] [1774883478.304757745] [camera.camera]: Sync Mode: Off
[realsense2_camera_node-1] [WARN] [1774883478.404697796] [camera.camera]: Could not set param: depth_module.power_line_frequency with 3 Range: [0, 2]: parameter 'depth_module.power_line_frequency' could not be set: Parameter {depth_module.power_line_frequency} doesn't comply with integer range.
[realsense2_camera_node-1] [INFO] [1774883478.443989143] [camera.camera]: Set ROS param depth_module.depth_profile to default: 848x480x30
[realsense2_camera_node-1] [INFO] [1774883478.444974206] [camera.camera]: Set ROS param depth_module.color_profile to default: 848x480x30
[realsense2_camera_node-1] [INFO] [1774883478.445746909] [camera.camera]: Set ROS param depth_module.infra_profile to default: 848x480x30
[realsense2_camera_node-1] [INFO] [1774883478.467308914] [camera.camera]: Stopping Sensor: Depth Module
[realsense2_camera_node-1] [INFO] [1774883478.482117180] [camera.camera]: Starting Sensor: Depth Module
[realsense2_camera_node-1] [INFO] [1774883478.507246174] [camera.camera]: Open profile: stream_type: Color(0), Format: RGB8, Width: 848, Height: 480, FPS: 30
[realsense2_camera_node-1] [INFO] [1774883478.507411269] [camera.camera]: Open profile: stream_type: Depth(0), Format: Z16, Width: 848, Height: 480, FPS: 30
[realsense2_camera_node-1] [INFO] [1774883478.516340038] [camera.camera]: RealSense Node Is Up!



ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true depth_module.depth_profile:=848x480x15 rgb_camera.color_profile:=848x480x15



ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true enable_sync:=true

rviz2 -d /workspace/src/custom_code/rviz_config.rviz

python3 /workspace/src/custom_packages/custom_code/template_matching_roi_icp_node.py