#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/segmented_image', 10)
        self.get_logger().info('Segmentation node with YOLO (Ultralytics) started')

        # Load YOLO model
        self.model = YOLO('yolov8n.pt')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO
            results = self.model(cv_image, verbose=False)
            result = results[0]

            # Draw bounding boxes and labels
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = f"{result.names[cls_id]} {conf:.2f}"

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Convert back to ROS Image
            segmented_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            segmented_msg.header = msg.header

            # Publish
            self.publisher.publish(segmented_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()