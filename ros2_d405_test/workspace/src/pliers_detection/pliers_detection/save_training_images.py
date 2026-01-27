import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        # Default topic for RealSense RGB
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('save_dir', 'training_data')
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info(f'Subscribed to {image_topic}')
        self.get_logger().info(f'Images will be saved to {os.path.abspath(self.save_dir)}')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Display info on the image
            display_img = cv_image.copy()
            cv2.putText(display_img, "Press 's' to save, 'q' to quit", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow("Data Collection", display_img)
            key = cv2.waitKey(1)
            
            if key & 0xFF == ord('s'):
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = os.path.join(self.save_dir, f"pliers_{timestamp}.jpg")
                cv2.imwrite(filename, cv_image) # Save original clean image
                self.get_logger().info(f"Saved {filename}")
                # Flash effect on save
                cv2.rectangle(display_img, (0,0), (display_img.shape[1], display_img.shape[0]), (255, 255, 255), 5)
                cv2.imshow("Data Collection", display_img)
                cv2.waitKey(50)
                
            elif key & 0xFF == ord('q'):
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    try:
        rclpy.spin(image_saver)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        image_saver.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
