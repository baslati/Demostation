#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoTfNode(Node):
    def __init__(self):
        super().__init__("aruco_tf_node")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("frame_prefix", "aruco_")

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        dict_name = self.get_parameter("dictionary").value
        self.frame_prefix = self.get_parameter("frame_prefix").value

        if dict_name not in DICT_MAP:
            self.get_logger().warn(f"Unbekanntes Dictionary {dict_name}, nutze DICT_4X4_50")
            dict_name = "DICT_4X4_50"

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[dict_name])
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        self.k = None
        self.dist = None
        self.camera_frame = "camera_color_optical_frame"

        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(f"Aruco TF Node läuft. Image: {image_topic}, CameraInfo: {camera_info_topic}")

    def _on_camera_info(self, msg: CameraInfo):
        if self.k is not None:
            return
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id or self.camera_frame
        self.get_logger().info(f"Kamera-Info erhalten, parent frame: {self.camera_frame}")

    def _on_image(self, msg: Image):
        if self.k is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.k, self.dist)

        for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f"{self.frame_prefix}{int(marker_id)}"

            t.transform.translation.x = float(tvec[0][0])
            t.transform.translation.y = float(tvec[0][1])
            t.transform.translation.z = float(tvec[0][2])

            rmat, _ = cv2.Rodrigues(rvec[0])
            qx, qy, qz, qw = self._quat_from_rotmat(rmat)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _quat_from_rotmat(r):
        trace = float(r[0, 0] + r[1, 1] + r[2, 2])
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (r[2, 1] - r[1, 2]) * s
            qy = (r[0, 2] - r[2, 0]) * s
            qz = (r[1, 0] - r[0, 1]) * s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s
        return float(qx), float(qy), float(qz), float(qw)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()