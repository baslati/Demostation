# ros2_d415_rviz_test

Minimaler Docker-Ordner zum Testen einer Intel RealSense D415 in ROS 2 Humble inkl. RViz.

## Was drin ist
- Dockerfile (Jetson/L4T Base wie in deinem `ros2_D415_project`)
- `docker-compose.yml` (host networking + USB passthrough + X11)
- EntryPoint, der `/opt/ros/humble` und `/workspace/install` sourced

## Start
1) Host: X11 erlauben (falls du noch nicht hast):

```bash
xhost +local:root
```

2) Container bauen + starten:

```bash
cd ros2_d415_rviz_test
./start.sh
```

## Im Container: D415 + RViz starten

```bash
# RealSense driver
ros2 launch realsense2_camera rs_launch.py \
  enable_pointcloud:=true \
  align_depth.enable:=true
```

In einem zweiten Container-Shell (oder im gleichen, wenn du es stoppst):

```bash
rviz2
```

**RViz Tipps**
- Add -> `PointCloud2` und Topic `/camera/camera/depth/color/points` (kann je nach config leicht abweichen)
- Add -> `Image` Topic `/camera/camera/color/image_raw`

## Troubleshooting
- Keine Kamera im Container: checke `lsusb` im Container und ob Compose `privileged: true` + `devices: /dev/bus/usb` gesetzt ist.
- GUI startet nicht: `DISPLAY` und `xhost +local:root` auf dem Host prüfen.

## Build-Lösung für OpenCV-Konflikte

Der ursprüngliche Build schlug fehl wegen Konflikten zwischen der OpenCV-Version im Base-Image (4.8.1-dirty) und den ROS-Paketen (cv_bridge basierend auf OpenCV 4.5). Die Lösung:

1. **Apt-Pinning**: Verhindert Installation von Ubuntu's OpenCV-Paketen, um Konflikte zu vermeiden.
   ```dockerfile
   RUN printf 'Package: libopencv*\nPin: release *\nPin-Priority: -1\n' > /etc/apt/preferences.d/block-ubuntu-opencv
   ```

2. **cv_bridge aus Source bauen**: Anstatt das vorkompilierte `ros-humble-cv-bridge` zu installieren, wird `vision_opencv` geklont und cv_bridge gegen die System-OpenCV 4.8 gebaut.
   ```dockerfile
   RUN mkdir -p /workspace/src && \
       git clone --branch humble https://github.com/ros-perception/vision_opencv.git /workspace/src/vision_opencv && \
       git clone --branch ros2-development https://github.com/IntelRealSense/realsense-ros.git /workspace/src/realsense-ros
   ```

3. **Linting-Tools hinzufügen**: `ament-lint-auto` und `ament-lint-common` werden installiert, um Build-Abhängigkeiten zu erfüllen.

Dies ermöglicht einen erfolgreichen Build ohne OpenCV-Konflikte, da cv_bridge gegen die korrekte OpenCV-Version kompiliert wird.
