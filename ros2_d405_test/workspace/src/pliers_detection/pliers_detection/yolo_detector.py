import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('confidence_threshold', 0.2)
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            10)
        
        # Publisher for the image with bounding boxes
        self.publisher_ = self.create_publisher(Image, '/pliers_detection/debug_image', 10)
        
        self.bridge = CvBridge()
        
        # Wir nutzen YOLO-World. Es laedt sich automatisch runter.
        # Es erlaubt uns, nach Begriffen zu suchen, ohne Training.
        self.get_logger().info('Lade YOLO-World Modell (kann beim ersten Mal kurz dauern)...')
        self.model = YOLO("yolov8s-world.pt") 
        self.model.set_classes(["pliers", "tool"]) # Wir definieren, was wir suchen!
        
        self.get_logger().info('YOLO Detector Node Initialized - Ready to detect Pliers')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # --- VISION PIPELINE START ---
            
            # Inference using YOLO
            # conf=0.1 setzt die Schwelle niedrig, damit wir erst mal 'irgendwas' sehen
            results = self.model.predict(cv_image, conf=0.15, verbose=False)
            
            # Zeichnet die Boxen direkt in das Bild
            annotated_frame = results[0].plot()
            
            # --- VISION PIPELINE END ---
            
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.publisher_.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in inference: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
