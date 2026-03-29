#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
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


def mat_to_quat(rot):
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rot[2, 1] - rot[1, 2]) * s
        qy = (rot[0, 2] - rot[2, 0]) * s
        qz = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        qw = (rot[2, 1] - rot[1, 2]) / s
        qx = 0.25 * s
        qy = (rot[0, 1] + rot[1, 0]) / s
        qz = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        qw = (rot[0, 2] - rot[2, 0]) / s
        qx = (rot[0, 1] + rot[1, 0]) / s
        qy = 0.25 * s
        qz = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        qw = (rot[1, 0] - rot[0, 1]) / s
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


class ArucoWorldTableFilterNode(Node):
    def __init__(self):
        super().__init__("aruco_world_table_filter_node")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("pointcloud_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("marker_size", 0.04)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_id", -1)
        self.declare_parameter("strict_marker_id", False)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("aruco_marker_frame", "aruco_marker")
        self.declare_parameter("camera_pose_frame", "camera_from_aruco")
        self.declare_parameter("filtered_topic", "/table_cropped_points")
        self.declare_parameter("pose_smooth_alpha", 0.2)
        self.declare_parameter("lock_world_after_init", True)
        self.declare_parameter("lock_after_detections", 20)
        self.declare_parameter("publish_full_if_crop_empty", True)
        self.declare_parameter("x_min", -0.20)
        self.declare_parameter("x_max", 1.20)
        self.declare_parameter("y_min", -0.20)
        self.declare_parameter("y_max", 1.20)
        self.declare_parameter("z_min", -0.10)
        self.declare_parameter("z_max", 0.30)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        dict_name = self.get_parameter("dictionary").value
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.strict_marker_id = bool(self.get_parameter("strict_marker_id").value)
        self.world_frame = self.get_parameter("world_frame").value
        self.aruco_marker_frame = self.get_parameter("aruco_marker_frame").value
        self.camera_pose_frame = self.get_parameter("camera_pose_frame").value
        filtered_topic = self.get_parameter("filtered_topic").value
        self.pose_smooth_alpha = float(self.get_parameter("pose_smooth_alpha").value)
        self.lock_world_after_init = bool(self.get_parameter("lock_world_after_init").value)
        self.lock_after_detections = int(self.get_parameter("lock_after_detections").value)
        self.publish_full_if_crop_empty = bool(self.get_parameter("publish_full_if_crop_empty").value)

        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)

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

        self.t_world_cam = None
        self.marker_visible = False
        self._fallback_logged = False
        self._smoothed_rvec = None
        self._smoothed_tvec = None
        self._last_ids_log_time = self.get_clock().now()
        self._last_t_world_cam = None
        self._last_crop_log_time = self.get_clock().now()
        self._world_locked = False
        self._world_lock_counter = 0
        self._locked_t_world_cam = None

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, qos_sensor)
        self.create_subscription(Image, image_topic, self._on_image, qos_sensor)
        self.create_subscription(PointCloud2, pointcloud_topic, self._on_pointcloud, qos_sensor)
        self.filtered_pub = self.create_publisher(PointCloud2, filtered_topic, 10)

        self.get_logger().info("Node gestartet: ArUco -> world + Tisch-Crop")
        self.get_logger().info(
            f"Crop (m): x[{self.x_min},{self.x_max}] y[{self.y_min},{self.y_max}] z[{self.z_min},{self.z_max}]")
        if self.lock_world_after_init:
            self.get_logger().info(
                f"World-Lock aktiv: fixiere world nach {self.lock_after_detections} ArUco-Detektionen")

    def _on_camera_info(self, msg):
        if self.k is not None:
            return
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id or self.camera_frame
        self.get_logger().info(f"Kamera-Info OK, frame: {self.camera_frame}")

    def _on_image(self, msg):
        if self.k is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            self.marker_visible = False
            return

        ids_flat = ids.flatten()
        now = self.get_clock().now()
        if (now - self._last_ids_log_time).nanoseconds > 2_000_000_000:
            self.get_logger().info(f"Aruco IDs erkannt: {ids_flat.tolist()}")
            self._last_ids_log_time = now

        if self.marker_id >= 0:
            selected = np.where(ids_flat == self.marker_id)[0]
            if len(selected) == 0:
                if self.strict_marker_id:
                    self.marker_visible = False
                    return
                index = 0
            else:
                index = int(selected[0])
        else:
            index = 0

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.k, self.dist)
        rvec = rvecs[index][0]
        tvec = tvecs[index][0]

        if self._smoothed_rvec is None:
            self._smoothed_rvec = rvec.copy()
            self._smoothed_tvec = tvec.copy()
        else:
            a = self.pose_smooth_alpha
            self._smoothed_rvec = (1.0 - a) * self._smoothed_rvec + a * rvec
            self._smoothed_tvec = (1.0 - a) * self._smoothed_tvec + a * tvec

        rvec = self._smoothed_rvec
        tvec = self._smoothed_tvec
        self.marker_visible = True
        self._fallback_logged = False

        rmat, _ = cv2.Rodrigues(rvec)
        # OpenCV liefert Marker->Kamera. Wir brauchen Kamera->World(=Marker) für Crop.
        t_cam_marker = np.eye(4, dtype=np.float64)
        t_cam_marker[:3, :3] = rmat
        t_cam_marker[:3, 3] = tvec
        t_world_cam = np.linalg.inv(t_cam_marker)
        self.t_world_cam = t_world_cam

        if self._world_locked:
            t_world_cam = self._locked_t_world_cam
        else:
            self._world_lock_counter += 1
            if self.lock_world_after_init and self._world_lock_counter >= self.lock_after_detections:
                self._world_locked = True
                self._locked_t_world_cam = t_world_cam.copy()
                t_world_cam = self._locked_t_world_cam
                self.get_logger().info("World-Frame fixiert (Aruco-Lock aktiv)")

        self._last_t_world_cam = t_world_cam
        self.t_world_cam = t_world_cam
        self._publish_camera_tfs(msg.header.stamp, t_world_cam)
        self._publish_marker_frame(msg.header.stamp)

    def _publish_marker_frame(self, stamp):
        marker_tf = TransformStamped()
        marker_tf.header.stamp = stamp
        marker_tf.header.frame_id = self.world_frame
        marker_tf.child_frame_id = self.aruco_marker_frame
        marker_tf.transform.translation.x = 0.0
        marker_tf.transform.translation.y = 0.0
        marker_tf.transform.translation.z = 0.0
        marker_tf.transform.rotation.x = 0.0
        marker_tf.transform.rotation.y = 0.0
        marker_tf.transform.rotation.z = 0.0
        marker_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(marker_tf)

    def _publish_camera_tfs(self, stamp, t_world_cam):
        # Connect world directly to the real camera frame so PointCloud TF chain is valid in RViz.
        tf_cam = TransformStamped()
        tf_cam.header.stamp = stamp
        tf_cam.header.frame_id = self.world_frame
        tf_cam.child_frame_id = self.camera_frame
        tf_cam.transform.translation.x = float(t_world_cam[0, 3])
        tf_cam.transform.translation.y = float(t_world_cam[1, 3])
        tf_cam.transform.translation.z = float(t_world_cam[2, 3])
        qx, qy, qz, qw = mat_to_quat(t_world_cam[:3, :3])
        tf_cam.transform.rotation.x = qx
        tf_cam.transform.rotation.y = qy
        tf_cam.transform.rotation.z = qz
        tf_cam.transform.rotation.w = qw

        if self.camera_pose_frame == self.camera_frame:
            self.tf_broadcaster.sendTransform(tf_cam)
            return

        tf_alias = TransformStamped()
        tf_alias.header.stamp = stamp
        tf_alias.header.frame_id = self.world_frame
        tf_alias.child_frame_id = self.camera_pose_frame
        tf_alias.transform.translation.x = tf_cam.transform.translation.x
        tf_alias.transform.translation.y = tf_cam.transform.translation.y
        tf_alias.transform.translation.z = tf_cam.transform.translation.z
        tf_alias.transform.rotation.x = tf_cam.transform.rotation.x
        tf_alias.transform.rotation.y = tf_cam.transform.rotation.y
        tf_alias.transform.rotation.z = tf_cam.transform.rotation.z
        tf_alias.transform.rotation.w = tf_cam.transform.rotation.w
        self.tf_broadcaster.sendTransform([tf_cam, tf_alias])

    def _make_world_cloud_msg(self, header, points_world, colors=None):
        header.frame_id = self.world_frame
        if colors is not None:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            xyz = points_world.astype(np.float32)
            data = np.column_stack((xyz, colors.reshape(-1, 1))).tolist()
            return pc2.create_cloud(header, fields, data)

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        xyz = points_world.astype(np.float32)
        return pc2.create_cloud(header, fields, xyz.tolist())

    def _on_pointcloud(self, msg):
        if self.t_world_cam is None or not self.marker_visible:
            if not self._fallback_logged:
                self.get_logger().info("Aruco nicht gefunden: publiziere gesamte PointCloud ungefiltert (letzte world-Lage bleibt)")
                self._fallback_logged = True
            if self._last_t_world_cam is not None:
                self._publish_camera_tfs(msg.header.stamp, self._last_t_world_cam)
                self._publish_marker_frame(msg.header.stamp)
            self.filtered_pub.publish(msg)
            return

        field_names = [f.name for f in msg.fields]
        color_field = None
        if "rgb" in field_names:
            color_field = "rgb"
        elif "rgba" in field_names:
            color_field = "rgba"

        read_fields = ("x", "y", "z", color_field) if color_field else ("x", "y", "z")
        raw_points = np.array(list(pc2.read_points(msg, field_names=read_fields, skip_nans=True)))
        colors = None
        if raw_points.dtype.fields is not None:
            points = np.column_stack((raw_points["x"], raw_points["y"], raw_points["z"]))
            points = points.astype(np.float64, copy=False)
            if color_field:
                colors = np.asarray(raw_points[color_field], dtype=np.float32)
        else:
            if color_field:
                points = np.asarray(raw_points[:, :3], dtype=np.float64)
                colors = np.asarray(raw_points[:, 3], dtype=np.float32)
            else:
                points = np.asarray(raw_points, dtype=np.float64)
        if points.size == 0:
            return

        rot = self.t_world_cam[:3, :3]
        trans = self.t_world_cam[:3, 3]
        points_world = (rot @ points.T).T + trans

        mask = (
            (points_world[:, 0] >= self.x_min) & (points_world[:, 0] <= self.x_max) &
            (points_world[:, 1] >= self.y_min) & (points_world[:, 1] <= self.y_max) &
            (points_world[:, 2] >= self.z_min) & (points_world[:, 2] <= self.z_max)
        )
        cropped = points_world[mask]
        now = self.get_clock().now()
        if (now - self._last_crop_log_time).nanoseconds > 2_000_000_000:
            self.get_logger().info(
                f"Crop stats: total={points_world.shape[0]} kept={cropped.shape[0]} "
                f"x[{self.x_min},{self.x_max}] y[{self.y_min},{self.y_max}] z[{self.z_min},{self.z_max}]")
            self._last_crop_log_time = now

        if cropped.shape[0] == 0:
            if self.publish_full_if_crop_empty:
                out_msg = self._make_world_cloud_msg(msg.header, points_world, colors)
                self.filtered_pub.publish(out_msg)
            return

        cropped_colors = colors[mask] if colors is not None else None

        out_msg = self._make_world_cloud_msg(msg.header, cropped, cropped_colors)
        self.filtered_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoWorldTableFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()