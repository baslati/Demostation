#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import cv2
import cv2.aruco as aruco
import numpy as np
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation')
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, '/vision/tool_pose_relative_to_marker', 10)
        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')  # Für Tool Detection, anpassen falls nötig
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 0.05  # 5cm Marker, anpassen
        self.latest_depth = None
        self.get_logger().info('Pose Estimation Node started.')

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {str(e)}')

    def color_callback(self, msg):
        if self.camera_matrix is None or self.latest_depth is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # ArUco Detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray)
            if ids is None or 0 not in ids:
                self.get_logger().warn('No ArUco marker ID 0 detected.')
                return
            index = np.where(ids == 0)[0][0]
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], self.marker_length, self.camera_matrix, self.dist_coeffs)
            rot_matrix = cv2.Rodrigues(rvec[0])[0]
            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = rot_matrix
            T_cam_marker[:3, 3] = tvec[0]
            
            # Tool Detection mit YOLO (angenommen 'pliers' Klasse, anpassen)
            results = self.yolo_model(cv_image)
            if not results[0].boxes:
                self.get_logger().warn('No tool detected.')
                return
            box = results[0].boxes[0]  # Erste Box
            center_x = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
            center_y = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
            depth = self.latest_depth[center_y, center_x] / 1000.0  # mm to m
            if depth == 0:
                self.get_logger().warn('Invalid depth at tool center.')
                return
            # Pose der Zange relativ zur Kamera (angenommen flach auf Tisch)
            T_cam_tool = np.eye(4)
            T_cam_tool[0, 3] = center_x
            T_cam_tool[1, 3] = center_y
            T_cam_tool[2, 3] = depth
            
            # T_marker_tool = inv(T_cam_marker) * T_cam_tool
            T_marker_tool = np.linalg.inv(T_cam_marker) @ T_cam_tool
            trans = T_marker_tool[:3, 3]
            rot_euler = R.from_matrix(T_marker_tool[:3, :3]).as_euler('xyz')
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'marker_0'
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]
            quat = R.from_matrix(T_marker_tool[:3, :3]).as_quat()
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            self.publisher.publish(pose_msg)
            self.get_logger().info(f'Published tool pose relative to marker: {trans}')
        except Exception as e:
            self.get_logger().error(f'Error in pose estimation: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()