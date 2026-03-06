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

