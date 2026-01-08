#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/vision/debug_img', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Lädt automatisch herunter falls nicht vorhanden
        self.get_logger().info('YOLO Detect Node started.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image)
            annotated_image = results[0].plot()  # Zeichnet Bounding Boxes
            debug_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.publisher.publish(debug_msg)
            self.get_logger().debug('Published debug image with detections.')
        except Exception as e:
            self.get_logger().error(f'Error in YOLO detection: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()