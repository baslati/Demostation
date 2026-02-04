# Image Segmentation BBoxes

This ROS 2 package performs real-time object detection using YOLOv8 (Ultralytics) and visualizes the results as a bgr8 image with bounding boxes and labels, which can be viewed in RViz.

## Setup

The package is part of the `ros2_d405_test` workspace. It depends on `ultralytics`.

## Usage Instructions

Follow these steps to build and run the segmentation node. All commands assume you are inside your Docker container.

### 1. Build the package

Open your terminal (inside the container) and run:

```bash
cd /workspace
colcon build --packages-select image_segmentation_bboxes
source install/setup.zsh
```

### 2. Start a new Terminal (Docker Exec)

If you need to open a new terminal session in the running container from your Mac terminal:

```bash
docker exec -it d405_test_container zsh
source /opt/ros/humble/setup.zsh
source /workspace/install/setup.zsh
```

### 3. Run the Segmentation Node

First, start your RealSense camera in a separate terminal (inside the container):

```bash
ros2 launch realsense2_camera rs_launch.py
```

Then, in your main terminal, start the segmentation node:

```bash
ros2 run image_segmentation_bboxes segmentation_node
```

### 4. Visualize in RViz

Start RViz (ensure you have X11 forwarding or a display setup):

```bash
rviz2
```

In RViz:
1. Click **Add** (bottom left).
2. Select **Image** and click **OK**.
3. Set the **Image Topic** to `/segmented_image`.
4. You should now see the camera feed with YOLO bounding boxes and labels.

## Topics

- **Subscribes to:** `/camera/color/image_raw` (sensor_msgs/Image)
- **Publishes to:** `/segmented_image` (sensor_msgs/Image)

## Important Notes

- **Internet Access**: On the first run, the node will automatically download `yolov8n.pt` from Ultralytics. Ensure your environment has internet access for this initial step.
- **Performance**: YOLOv8n is used for better performance on edge devices like the Jetson.
