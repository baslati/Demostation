# ROS 2 D415 Test (Seeed Jetson Orin)

## Docker-Image bauen

Im Verzeichnis `ros2_D415_test` ausführen:

```bash
docker compose build
```

## Container starten

```bash
docker compose up
```
Der Container läuft dann im interaktiven Modus mit GPU- und USB-Zugriff.

## Kamera testen (im Container)

```bash
lsusb
# Die D415 sollte als Intel Corp. RealSense 3D Camera erscheinen
```

## RealSense-Kamera-Node starten

```bash
ros2 launch realsense2_camera rs_launch.py
```

## RViz öffnen (neues Terminal im Container)

```bash
rviz2
```
In RViz das Topic `/camera/color/image_raw` auswählen, um das Kamerabild zu sehen.

## Alternative: RealSense-Tools direkt nutzen

```bash
realsense-viewer
# oder
rs-enumerate-devices
```
Hinweis: realsense-viewer benötigt ggf. X11-Weiterleitung.
