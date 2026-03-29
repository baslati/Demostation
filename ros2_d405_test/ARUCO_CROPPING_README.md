# ArUco Marker Point Cloud Cropping - Dokumentation

## Überblick

Das Programm `aruco_tf_node.py` wurde erweitert, um einen interaktiven Multi-Step-Workflow für Point Cloud Cropping basierend auf ArUco-Markern zu implementieren.

## Workflow-Schritte

### Schritt 1: Vollständige Point Cloud publishen
- Das Programm startet und publiziert die gesamte Point Cloud live auf dem Topic `/cloud_full`
- Die ArUco-Transform wird mit TF2 gesendet
- Status in RViz: Siehe die komplette Punktwolke live

### Schritt 2: Standaufnahme erstellen
- Drücke **ENTER** wenn die Standaufnahme bereit sein soll
- Der aktuelle Frame der Point Cloud wird als Snapshot gespeichert
- Das Programm geht in den nächsten Schritt über

### Schritt 3: Crop-Bereich eingeben
- Gib den XYZ-Bereich relativ zum ArUco-Marker ein
- Format: `x_min x_max y_min y_max z_min z_max` (in Metern)
- Beispiel: `-0.1 0.1 -0.1 0.1 0 0.3`
  - X-Bereich: -10cm bis +10cm
  - Y-Bereich: -10cm bis +10cm
  - Z-Bereich: 0cm bis 30cm (nach oben vom Marker)

### Schritt 4: Crop-Vorschau anzeigen
- Der Crop wird angezeigt und auf Topic `/cloud_cropped` publiziert
- In RViz kannst du das gecroppte Ergebnis sehen

### Schritt 5: Bestätigung des Crops
- Gib `j` ein wenn der Crop passt
- Gib `n` ein wenn ein neuer Crop-Bereich eingegeben werden soll (zurück zu Schritt 3)

### Schritt 6: Continuous Publishing
- Nach Bestätigung publisht das Programm fortlaufend die gecroppte Point Cloud
- Das ermöglicht Echtzeit-Verarbeitung der gecroppten Cloud
- Drücke `q` um das Programm zu beenden

## Parameter

Das Programm verwendet folgende ROS2-Parameter (in der launch-Datei oder beim Starten):

```
  --ros-args
    -p image_topic:=/camera/camera/color/image_raw
    -p camera_info_topic:=/camera/camera/color/camera_info
    -p pointcloud_topic:=/camera/depth/color/points
    -p marker_size:=0.04
    -p dictionary:=DICT_4X4_50
    -p frame_prefix:=aruco_
    -p marker_id_for_crop:=0
```

### Wichtige Parameter:
- **pointcloud_topic**: Muss die Point Cloud 2 Message vom D405 sein
- **marker_size**: Größe des ArUco-Markers in Metern
- **marker_id_for_crop**: Die ID des Markers, der als Referenz für Cropping verwendet wird

## Koordinaten-System

Das Crop-Bereich-Koordinatensystem ist:
- **Origin**: Position des ArUco-Markers
- **X-Achse**: Horizontale Achse im Marker-Frame (rechts/links)
- **Y-Achse**: Vertikal gemäß Marker-Orientierung (auf/ab)
- **Z-Achse**: Tiefe vom Marker weg (näher/ferner)

Die Transformation wird automatisch durchgeführt, da der Marker auch als Fixed Frame in RViz gesetzt wird.

## Topics

### Input:
- `/camera/camera/color/image_raw` - RGB-Bild vom D405
- `/camera/camera/color/camera_info` - Kamera-Instrinsics
- `/camera/depth/color/points` - Point Cloud 2 vom D405

### Output:
- `/cloud_full` - Vollständige Point Cloud (während Schritt 1)
- `/cloud_cropped` - Gecroppte Point Cloud (während Schritt 4-6)
- `/tf` - Transforms für ArUco-Marker (kontinuierlich)

## RViz Setup

1. Definiere folgende Displays:
   - **PointCloud2** für `/cloud_full` (ganz, grün)
   - **PointCloud2** für `/cloud_cropped` (rot/anders farbig)
   - **TF** um Transform-Frames zu sehen
   
2. Setze den **Fixed Frame** auf den ArUco-Marker:
   - Z.B. `aruco_0` wenn `marker_id_for_crop:=0` und `frame_prefix:=aruco_`

3. Positioniere die Ansicht so, dass du den Crop-Bereich (Bounding Box) visualisieren kannst

## Beispiel-Verwendung

```bash
# Terminal 1: Starte den Node
ros2 run custom_code aruco_tf_node \
  --ros-args \
    -p marker_id_for_crop:=0 \
    -p marker_size:=0.04

# Terminal 2: Starte RViz
rviz2

# Terminal 3: Monitor Topics
ros2 topic echo /cloud_cropped | head -20
```

## Workflow-Beispiel in der Konsole

```
============================================================
Schritt 1: Veröffentliche vollständige Point Cloud
Drücke ENTER wenn Standaufnahme bereit ist...
============================================================

[User drückt ENTER]

>>> Standaufnahme wird erstellt...
============================================================
Schritt 2: Standaufnahme gespeichert
Gib XYZ Bereich ein (in Meter, relativ zum Marker):
Format: x_min x_max y_min y_max z_min z_max
Beispiel: -0.1 0.1 -0.1 0.1 0 0.3
============================================================

-0.15 0.15 -0.15 0.15 0 0.4

>>> Crop Grenzen gesetzt: (-0.15, 0.15, -0.15, 0.15, 0, 0.4)
============================================================
Schritt 3: Cropped Point Cloud angezeigt
Passt der Crop? (j/n)
============================================================

j

>>> Crop akzeptiert! Starte Publikation der gecroppten Cloud...
============================================================
Schritt 4: Veröffentliche cropped Point Cloud
Drücke 'q' um Programm zu beenden...
============================================================

[Programm läuft und publisht gecroppte Cloud...]

q

>>> Programm wird beendet...
```

## Troubleshooting

### "Keine Punkte im Crop-Bereich!"
- Die Crop-Grenzen sind zu restriktiv
- Vergrößere die Grenzen (z.B. -0.3 statt -0.15)
- Überprüfe, ob der Marker richtig erkannt wird

### ArUco-Marker wird nicht erkannt
- Beleuchtung überprüfen
- Marker-Größe muss mit `marker_size` Parameter übereinstimmen
- Marker-ID muss mit `marker_id_for_crop` Parameter übereinstimmen
- Dictionary muss korrekt gesetzt sein

### Point Cloud ist leer
- Überprüfe, ob D405 Point Cloud publiziert
- Mit `ros2 topic list` oder `ros2 topic echo /camera/depth/color/points` testen

## Technische Änderungen gegenüber Original

1. **WorkflowState Enum**: Kontrolliert den Multi-Step Prozess
2. **Threading**: Benutzer-Eingaben laufen in separatem Thread
3. **Point Cloud Processing**:
   - `_pointcloud2_to_array()`: Konvertiert ROS MSG zu NumPy Array
   - `_array_to_pointcloud2()`: Konvertiert Array zurück zu ROS MSG
   - `_crop_pointcloud()`: Führt Cropping mit Marker-Frame-Transformation durch

4. **Publishers**: Zwei neue Publisher für full und cropped clouds
5. **Marker Tracking**: Speichert Position und Rotation des Target-Markers für Transformationen
