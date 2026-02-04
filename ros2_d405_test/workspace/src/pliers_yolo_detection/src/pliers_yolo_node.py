#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray

class PliersYoloNode(Node):
    def __init__(self):
        super().__init__('pliers_yolo_node')

        # Parameters
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('target_class_id', 76)  # 76 = scissors (proxy for pliers)

        self.model_path = self.get_parameter('yolo_model').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.target_class = self.get_parameter('target_class_id').value

        # YOLO Model
        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.model = YOLO(self.model_path)

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.debug_pub = self.create_publisher(Image, '/pliers/debug_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pliers/markers', 10)

        # State
        self.latest_color = None
        self.latest_depth = None
        self.camera_intrinsics = None

        self.get_logger().info('Pliers YOLO Node Ready.')

    def camera_info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = np.array(msg.k).reshape(3, 3)
            self.get_logger().info('Camera Intrinsics received.')

    def depth_callback(self, msg):
        self.get_logger().info('Depth msg received', throttle_duration_sec=5.0)
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def color_callback(self, msg):
        self.get_logger().info('Color msg received', throttle_duration_sec=5.0)
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f'Color callback error: {e}')

    def process_frame(self):
        if self.latest_color is None or self.latest_depth is None or self.camera_intrinsics is None:
            self.get_logger().warn(
                f'Missing data! Color: {self.latest_color is not None}, '
                f'Depth: {self.latest_depth is not None}, '
                f'Intrinsics: {self.camera_intrinsics is not None}',
                throttle_duration_sec=2.0
            )
            return

        # Run YOLO
        results = self.model(self.latest_color, verbose=False)
        result = results[0]

        marker_array = MarkerArray()
        debug_img = self.latest_color.copy()

        detection_id = 0
        boxes = result.boxes
        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            if conf < self.conf_thresh:
                continue

            # Bounding Box
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Center
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # Get depth at center (robust with median)
            h, w = self.latest_depth.shape
            cx = max(0, min(cx, w-1))
            cy = max(0, min(cy, h-1))

            depth_region = self.latest_depth[max(0, cy-5):min(h, cy+5), max(0, cx-5):min(w, cx+5)]
            if depth_region.size == 0:
                continue

            valid_depths = depth_region[depth_region > 0]
            if len(valid_depths) == 0:
                continue

            depth_mm = np.median(valid_depths)
            depth_m = depth_mm / 1000.0

            # Deproject to 3D
            fx = self.camera_intrinsics[0, 0]
            fy = self.camera_intrinsics[1, 1]
            ppx = self.camera_intrinsics[0, 2]
            ppy = self.camera_intrinsics[1, 2]

            X = (cx - ppx) * depth_m / fx
            Y = (cy - ppy) * depth_m / fy
            Z = depth_m

            # Estimate box size
            obj_width = (x2 - x1) * depth_m / fx
            obj_height = (y2 - y1) * depth_m / fy
            obj_thickness = 0.05  # 5cm assumed

            # 3D Marker
            marker = Marker()
            marker.header.frame_id = "camera_color_optical_frame"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = detection_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = X
            marker.pose.position.y = Y
            marker.pose.position.z = Z
            marker.scale.x = obj_width
            marker.scale.y = obj_height
            marker.scale.z = obj_thickness
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.lifetime.sec = 1

            marker_array.markers.append(marker)
            detection_id += 1

            # Draw on debug image
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{result.names[cls_id]} {conf:.2f} {depth_m:.2f}m"
            cv2.putText(debug_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish
        self.marker_pub.publish(marker_array)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PliersYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()