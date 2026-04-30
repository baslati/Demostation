#!/usr/bin/env python3
"""
Template Scan + Grasp Point workflow (D405-only).

Ablauf:
1) ArUco-Snapshot + Cropping + Speicherung als PCD
2) Offline-Preprocessing zum Template
3) Gripper-Offset-Kalibrierung mit manueller Greifpose (Pose via ROS Topic)

Hinweis: Keine Robotersteuerung/MoveIt. Die manuelle TCP-Pose wird von einem
externen Topic geliefert (default: /tcp_pose_broadcaster/pose).
"""

import copy
import io
import math
import os
import struct
import threading
import time
from datetime import datetime
from enum import Enum
from typing import Optional, Tuple

import cv2
import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


TARGET_TOPIC = "/tool_target_pose"
DEFAULT_TEMPLATE_ID = "cropv1_clean_direction"

SERVICE_TIMEOUT_SEC = 8.0
HOLD_SECONDS = 2.0

GRIPPER_FIXED_ROLL = math.pi
GRIPPER_FIXED_PITCH = 0.0
TOOL_YAW_OFFSET = 0.0

TARGET_POSE_IS_TABLE_COORDS = True
TABLE_ORIGIN_IN_BASE_X_M = -0.15
TABLE_ORIGIN_IN_BASE_Y_M = 0.15
TABLE_ORIGIN_IN_BASE_Z_M = 0.0
TABLE_TO_BASE_X_SIGN = -1.0
TABLE_TO_BASE_Y_SIGN = -1.0
TABLE_TO_BASE_Z_SIGN = 1.0

ARUCO_IN_BASE_X_M = 0.15
ARUCO_IN_BASE_Y_M = 0.15
ARUCO_IN_BASE_Z_M = 0.0
ARUCO_IN_BASE_RX = 0.0
ARUCO_IN_BASE_RY = 0.0
ARUCO_IN_BASE_RZ = 0.0

# Integrated one-shot ROI/ICP scan settings.
TOOL_SCAN_TIMEOUT_SEC = 12.0
MIN_CLOUD_STAMP_AFTER_SCAN_SEC = 0.10
MATCH_INTERVAL_SEC = 0.25
MAX_RAW_POINTS = 60000
VOXEL_SIZE_SCENE = 0.0025
VOXEL_SIZE_TEMPLATE = 0.0025
PLANE_DISTANCE_THRESHOLD = 0.0015
PLANE_RANSAC_ITER = 200
PLANE_MIN_KEEP_POINTS = 120
CLUSTER_EPS = 0.012
CLUSTER_MIN_POINTS = 30
MIN_CLUSTER_POINTS = 90
MAX_CLUSTERS_TO_TEST = 4
ICP_THRESHOLD = 0.010
ICP_MAX_ITER = 45
MIN_SEARCH_FITNESS = 0.60
MAX_SEARCH_RMSE = 0.10
POINT_COUNT_RATIO_MIN = 0.35
POINT_COUNT_RATIO_MAX = 1.80
PCA_EXTENT_RATIO_MIN = 0.45
PCA_EXTENT_RATIO_MAX = 1.90

DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class WorkflowState(Enum):
    SNAPSHOT = 1
    PREPROCESS = 2
    CALIBRATE = 3
    DONE = 4


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    x2 = qx + qx
    y2 = qy + qy
    z2 = qz + qz

    xx = qx * x2
    yy = qy * y2
    zz = qz * z2
    xy = qx * y2
    xz = qx * z2
    yz = qy * z2
    wx = qw * x2
    wy = qw * y2
    wz = qw * z2

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def rotmat_to_quat(r: np.ndarray) -> Tuple[float, float, float, float]:
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


def rotmat_to_rpy(r: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(r[0, 0] * r[0, 0] + r[1, 0] * r[1, 0])
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(r[2, 1], r[2, 2])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def rpy_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def pca_rotation(points: np.ndarray) -> np.ndarray:
    centered = points - points.mean(axis=0)
    cov = centered.T @ centered / max(1, len(points) - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    r = eigvecs[:, order]
    if np.linalg.det(r) < 0.0:
        r[:, 2] *= -1.0
    return r


def rot_z(theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def normalize_frame_id(frame_id: str) -> str:
    return frame_id.strip().lstrip("/")


def split_frame_and_template(frame_id: str) -> Tuple[str, str]:
    norm = normalize_frame_id(frame_id)
    if "|" not in norm:
        return norm, DEFAULT_TEMPLATE_ID
    base_frame, template_id = norm.split("|", 1)
    base_frame = base_frame.strip()
    template_id = template_id.strip() or DEFAULT_TEMPLATE_ID
    return base_frame, template_id


def safe_slug(value: str) -> str:
    cleaned = [c if c.isalnum() or c in ("-", "_") else "_" for c in value.strip()]
    slug = "".join(cleaned).strip("_")
    return slug or "template"


def ensure_parent_dir(path: str) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)


def remove_table_ransac(pcd: o3d.geometry.PointCloud, distance_threshold=0.005, iterations=1000):
    if len(pcd.points) < 100:
        return pcd

    _, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=iterations,
    )
    return pcd.select_by_index(inliers, invert=True)


def extract_largest_cluster(pcd: o3d.geometry.PointCloud, eps=0.005, min_points=100):
    if len(pcd.points) < min_points:
        return pcd

    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    if len(labels) == 0 or labels.max() < 0:
        return pcd

    cluster_sizes = []
    for i in range(labels.max() + 1):
        cluster_sizes.append(int(np.sum(labels == i)))

    largest_idx = int(np.argmax(cluster_sizes))
    largest_mask = labels == largest_idx
    return pcd.select_by_index(np.where(largest_mask)[0].tolist())


def remove_outliers(pcd: o3d.geometry.PointCloud, nb_neighbors=20, std_ratio=2.0):
    if len(pcd.points) < nb_neighbors:
        return pcd

    filtered, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )
    return filtered


def voxel_downsample(pcd: o3d.geometry.PointCloud, voxel_size=0.001):
    return pcd.voxel_down_sample(voxel_size=voxel_size)


def compute_centroid_and_translate(pcd: o3d.geometry.PointCloud):
    points = np.asarray(pcd.points, dtype=np.float64)
    centroid = points.mean(axis=0)
    pcd.translate(-centroid, relative=True)
    return pcd, centroid


def align_y_axis_to_tool(pcd: o3d.geometry.PointCloud):
    pts = np.asarray(pcd.points, dtype=np.float64)
    if len(pts) < 10:
        return pcd

    centered = pts - pts.mean(axis=0)
    cov = centered.T @ centered / max(1, len(pts) - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    basis = eigvecs[:, order]

    if np.linalg.det(basis) < 0.0:
        basis[:, 2] *= -1.0

    longest = basis[:, 0]
    middle = basis[:, 1]

    if longest[1] < 0.0:
        longest = -longest

    z_axis = np.cross(middle, longest)
    z_norm = np.linalg.norm(z_axis)
    if z_norm < 1e-9:
        return pcd
    z_axis = z_axis / z_norm

    x_axis = np.cross(longest, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = longest / np.linalg.norm(longest)

    world_from_local = np.column_stack((x_axis, y_axis, z_axis))
    rot = world_from_local.T
    pcd.rotate(rot, center=np.array([0.0, 0.0, 0.0]))
    return pcd


def preprocess_template_cloud(
    pcd: o3d.geometry.PointCloud,
    ransac_threshold=0.004,
    ransac_iterations=1000,
    cluster_eps=0.010,
    cluster_min_points=100,
    outlier_neighbors=20,
    outlier_std=2.0,
    voxel_size=0.001,
    downsample=True,
):
    pcd = remove_table_ransac(pcd, ransac_threshold, ransac_iterations)
    if len(pcd.points) == 0:
        raise RuntimeError("Keine Punkte nach Tisch-Entfernung uebrig")

    pcd = extract_largest_cluster(pcd, cluster_eps, cluster_min_points)
    if len(pcd.points) == 0:
        raise RuntimeError("Keine Punkte nach Clustering uebrig")

    pcd = remove_outliers(pcd, outlier_neighbors, outlier_std)
    if downsample:
        pcd = voxel_downsample(pcd, voxel_size)

    pcd, centroid = compute_centroid_and_translate(pcd)
    pcd = align_y_axis_to_tool(pcd)
    return pcd, centroid


def pcd_has_points(pcd: Optional[o3d.geometry.PointCloud]) -> bool:
    return pcd is not None and len(pcd.points) > 0


class TemplateScanGraspPoint(Node):
    def __init__(self) -> None:
        super().__init__("template_scan_grasp_point_d405")

        script_dir = os.path.dirname(os.path.abspath(__file__))

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("pointcloud_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("tcp_pose_topic", "/tcp_pose_broadcaster/pose")
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("frame_prefix", "aruco_")
        self.declare_parameter("marker_id_for_crop", 0)
        self.declare_parameter("snapshot_save_dir", os.path.join(script_dir, "scans"))
        self.declare_parameter("template_save_dir", os.path.join(script_dir, "templates"))
        self.declare_parameter("offset_save_dir", os.path.join(script_dir, "templates"))
        self.declare_parameter("crop_bounds_default", "-0.15 0.15 -0.10 0.10 0.0 0.30")
        self.declare_parameter("ransac_threshold", 0.004)
        self.declare_parameter("ransac_iterations", 1000)
        self.declare_parameter("cluster_eps", 0.010)
        self.declare_parameter("cluster_min_points", 100)
        self.declare_parameter("voxel_size", 0.001)
        self.declare_parameter("outlier_neighbors", 20)
        self.declare_parameter("outlier_std", 2.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.tcp_pose_topic = self.get_parameter("tcp_pose_topic").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        dict_name = self.get_parameter("dictionary").value
        self.frame_prefix = self.get_parameter("frame_prefix").value
        self.target_marker_id = int(self.get_parameter("marker_id_for_crop").value)
        self.snapshot_save_dir = self.get_parameter("snapshot_save_dir").value
        self.template_save_dir = self.get_parameter("template_save_dir").value
        self.offset_save_dir = self.get_parameter("offset_save_dir").value
        self.crop_bounds_default = self.get_parameter("crop_bounds_default").value
        self.ransac_threshold = float(self.get_parameter("ransac_threshold").value)
        self.ransac_iterations = int(self.get_parameter("ransac_iterations").value)
        self.cluster_eps = float(self.get_parameter("cluster_eps").value)
        self.cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.outlier_neighbors = int(self.get_parameter("outlier_neighbors").value)
        self.outlier_std = float(self.get_parameter("outlier_std").value)

        if dict_name not in DICT_MAP:
            self.get_logger().warn(f"Unbekanntes Dictionary {dict_name}, nutze DICT_4X4_50")
            dict_name = "DICT_4X4_50"

        os.makedirs(self.snapshot_save_dir, exist_ok=True)
        os.makedirs(self.template_save_dir, exist_ok=True)
        os.makedirs(self.offset_save_dir, exist_ok=True)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[dict_name])
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        self.base = "base_link"

        self.k = None
        self.dist = None
        self.camera_frame = "camera_color_optical_frame"

        self.latest_target_pose: Optional[PoseStamped] = None
        self.latest_template_id = DEFAULT_TEMPLATE_ID
        self.target_lock = threading.Lock()
        self.target_event = threading.Event()

        self.latest_tcp_pose: Optional[PoseStamped] = None
        self.tcp_lock = threading.Lock()
        self.tcp_event = threading.Event()

        self.current_pointcloud: Optional[PointCloud2] = None
        self.latest_live_pointcloud: Optional[PointCloud2] = None
        self.snapshot_pointcloud: Optional[PointCloud2] = None
        self.snapshot_marker_position = None
        self.snapshot_marker_rotation = None
        self.marker_position = None
        self.marker_rotation = None
        self.cached_cropped_pointcloud: Optional[PointCloud2] = None
        self.crop_bounds = None
        self.lock = threading.Lock()
        self._got_first_cloud = False
        self._shutdown = False
        self._snapshot_taken = False
        self.template_points: Optional[np.ndarray] = None
        self.template_point_count = 0
        self.template_pca_extent: Optional[np.ndarray] = None

        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, qos_pose)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_pose)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self._on_pointcloud, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, TARGET_TOPIC, self._on_target_pose, qos_pose)
        self.create_subscription(PoseStamped, self.tcp_pose_topic, self._on_tcp_pose, qos_pose)

        self.pub_cropped_cloud = self.create_publisher(PointCloud2, "/cloud_cropped", 10)
        self.pub_full_cloud = self.create_publisher(PointCloud2, "/cloud_full", 10)
        self.pub_axes_marker = self.create_publisher(MarkerArray, "/snapshot_axes", 10)
        self.pub_target_pose = self.create_publisher(PoseStamped, TARGET_TOPIC, 10)

        self.publish_timer = self.create_timer(1.0, self._publish_outputs_timer)

        self.workflow_thread = threading.Thread(target=self._workflow, daemon=True)
        self.workflow_thread.start()

        self.get_logger().info("Template Scan + Grasp Point gestartet (D405-only)")
        self.get_logger().info(f"  Image: {self.image_topic}")
        self.get_logger().info(f"  PointCloud: {self.pointcloud_topic}")
        self.get_logger().info(f"  Target topic: {TARGET_TOPIC}")
        self.get_logger().info(f"  TCP pose topic: {self.tcp_pose_topic}")
        self.get_logger().info(f"  Snapshot dir: {self.snapshot_save_dir}")
        self.get_logger().info(f"  Template dir: {self.template_save_dir}")
        self.get_logger().info(f"  Offset dir: {self.offset_save_dir}")

    def _wait_future_result(self, fut, label: str, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.01)
        if not fut.done():
            raise RuntimeError(f"Timeout bei {label} nach {timeout_sec:.1f}s")
        exc = fut.exception()
        if exc is not None:
            raise RuntimeError(f"{label} Exception: {exc}")
        return fut.result()

    def _on_target_pose(self, msg: PoseStamped) -> None:
        frame, template_id = split_frame_and_template(msg.header.frame_id)
        msg.header.frame_id = frame
        with self.target_lock:
            self.latest_target_pose = msg
            self.latest_template_id = template_id
        self.target_event.set()

    def _on_tcp_pose(self, msg: PoseStamped) -> None:
        with self.tcp_lock:
            self.latest_tcp_pose = msg
        self.tcp_event.set()

    def _target_in_base(self, msg: PoseStamped) -> Tuple[np.ndarray, Quaternion]:
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        frame = normalize_frame_id(msg.header.frame_id)
        q = msg.pose.orientation
        r_target = quat_to_rotmat(q.x, q.y, q.z, q.w)

        if frame == self.base:
            q_out = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
            return p, q_out

        if not frame.startswith(self.frame_prefix):
            raise RuntimeError(f"Unbekannter Eingangsframe: {msg.header.frame_id}")

        if TARGET_POSE_IS_TABLE_COORDS:
            p_base = np.array(
                [
                    TABLE_ORIGIN_IN_BASE_X_M + (TABLE_TO_BASE_X_SIGN * p[0]),
                    TABLE_ORIGIN_IN_BASE_Y_M + (TABLE_TO_BASE_Y_SIGN * p[1]),
                    TABLE_ORIGIN_IN_BASE_Z_M + (TABLE_TO_BASE_Z_SIGN * p[2]),
                ],
                dtype=np.float64,
            )
            q_out = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
            return p_base, q_out

        r_base_aruco = rpy_to_rotmat(ARUCO_IN_BASE_RX, ARUCO_IN_BASE_RY, ARUCO_IN_BASE_RZ)
        p_base_aruco = np.array([ARUCO_IN_BASE_X_M, ARUCO_IN_BASE_Y_M, ARUCO_IN_BASE_Z_M], dtype=np.float64)

        p_base = p_base_aruco + (r_base_aruco @ p)
        r_base_target = r_base_aruco @ r_target
        qx, qy, qz, qw = rotmat_to_quat(r_base_target)
        q_out = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return p_base, q_out

    def _wait_for_target_pose(self) -> Tuple[PoseStamped, str]:
        self.target_event.clear()
        self.get_logger().info(f"Warte auf Pose auf {TARGET_TOPIC}...")
        while rclpy.ok() and not self._shutdown:
            if self.target_event.wait(timeout=0.2):
                with self.target_lock:
                    pose = self.latest_target_pose
                    template_id = self.latest_template_id
                if pose is not None:
                    return pose, template_id
        raise RuntimeError("Node wird beendet")

    def _wait_for_tcp_pose(self, label: str) -> PoseStamped:
        self.tcp_event.clear()
        self.get_logger().info(f"Warte auf TCP Pose auf {self.tcp_pose_topic} ({label})...")
        while rclpy.ok() and not self._shutdown:
            if self.tcp_event.wait(timeout=0.2):
                with self.tcp_lock:
                    pose = self.latest_tcp_pose
                if pose is not None:
                    return pose
        raise RuntimeError("Node wird beendet")

    def _format_pose(self, label: str, pose: PoseStamped) -> str:
        p = pose.pose.position
        q = pose.pose.orientation
        r = quat_to_rotmat(q.x, q.y, q.z, q.w)
        roll, pitch, yaw = rotmat_to_rpy(r)
        return (
            f"{label}: frame={pose.header.frame_id}, "
            f"pos=({p.x:+.5f}, {p.y:+.5f}, {p.z:+.5f}) m, "
            f"quat=({q.x:+.6f}, {q.y:+.6f}, {q.z:+.6f}, {q.w:+.6f}), "
            f"rpy=({roll:+.5f}, {pitch:+.5f}, {yaw:+.5f}) rad"
        )

    def _compute_offset(self, detected_base: PoseStamped, manual_base: PoseStamped):
        pd = np.array(
            [
                detected_base.pose.position.x,
                detected_base.pose.position.y,
                detected_base.pose.position.z,
            ],
            dtype=np.float64,
        )
        qd = detected_base.pose.orientation
        rd = quat_to_rotmat(qd.x, qd.y, qd.z, qd.w)

        pm = np.array(
            [
                manual_base.pose.position.x,
                manual_base.pose.position.y,
                manual_base.pose.position.z,
            ],
            dtype=np.float64,
        )
        qm = manual_base.pose.orientation
        rm = quat_to_rotmat(qm.x, qm.y, qm.z, qm.w)

        p_off = rd.T @ (pm - pd)
        r_off = rd.T @ rm
        qx, qy, qz, qw = rotmat_to_quat(r_off)
        rr, rp, ry = rotmat_to_rpy(r_off)
        return p_off, (qx, qy, qz, qw), (rr, rp, ry)

    def _format_yaml_text(
        self,
        template_id: str,
        snapshot_pcd: str,
        template_pcd: str,
        crop_bounds: Tuple[float, float, float, float, float, float],
        offset_translation: np.ndarray,
        offset_quat: Tuple[float, float, float, float],
    ) -> str:
        return "\n".join(
            [
                f"template_id: {template_id}",
                f"snapshot_pcd: {snapshot_pcd}",
                f"template_pcd: {template_pcd}",
                "crop_bounds_marker:",
                f"  x_min: {crop_bounds[0]:+.6f}",
                f"  x_max: {crop_bounds[1]:+.6f}",
                f"  y_min: {crop_bounds[2]:+.6f}",
                f"  y_max: {crop_bounds[3]:+.6f}",
                f"  z_min: {crop_bounds[4]:+.6f}",
                f"  z_max: {crop_bounds[5]:+.6f}",
                "grasp_offset:",
                f"  translation_xyz_m: [{offset_translation[0]:+.6f}, {offset_translation[1]:+.6f}, {offset_translation[2]:+.6f}]",
                f"  rotation_quat_xyzw: [{offset_quat[0]:+.6f}, {offset_quat[1]:+.6f}, {offset_quat[2]:+.6f}, {offset_quat[3]:+.6f}]",
                f"generated_at: {datetime.now().isoformat(timespec='seconds')}",
            ]
        )

    def _save_text_file(self, path: str, content: str) -> str:
        ensure_parent_dir(path)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(content)
            handle.write("\n")
        return path

    def _save_pointcloud(self, path: str, pcd: o3d.geometry.PointCloud) -> str:
        ensure_parent_dir(path)
        if not o3d.io.write_point_cloud(path, pcd):
            raise RuntimeError(f"Konnte Punktwolke nicht speichern: {path}")
        return path

    def _load_template_for_scan(self, template_path: str) -> None:
        pcd = o3d.io.read_point_cloud(template_path)
        if len(pcd.points) < 50:
            raise RuntimeError(f"Template ungueltig oder zu klein: {template_path}")

        pcd = pcd.voxel_down_sample(VOXEL_SIZE_TEMPLATE)
        pts = np.asarray(pcd.points, dtype=np.float64)
        centroid = pts.mean(axis=0)
        self.template_points = pts - centroid
        self.template_point_count = len(self.template_points)
        self.template_pca_extent = self._pca_extent(self.template_points)

        self.get_logger().info(
            f"Template fuer Scan geladen: {template_path} ({self.template_point_count} Punkte)"
        )

    def _pca_extent(self, points: np.ndarray) -> np.ndarray:
        if len(points) < 10:
            return np.array([1e-6, 1e-6, 1e-6], dtype=np.float64)
        r = pca_rotation(points)
        centered = points - points.mean(axis=0)
        proj = centered @ r
        ext = np.ptp(proj, axis=0)
        return np.maximum(ext, 1e-6)

    def _cluster_candidates(self, pcd: o3d.geometry.PointCloud):
        labels = np.array(pcd.cluster_dbscan(eps=CLUSTER_EPS, min_points=CLUSTER_MIN_POINTS, print_progress=False))
        if len(labels) == 0 or labels.max() < 0:
            return [pcd]

        candidates = []
        for i in range(labels.max() + 1):
            idx = np.where(labels == i)[0]
            if len(idx) < MIN_CLUSTER_POINTS:
                continue
            candidates.append(pcd.select_by_index(idx.tolist()))

        if not candidates:
            return [pcd]
        candidates.sort(key=lambda c: len(c.points), reverse=True)
        return candidates[:MAX_CLUSTERS_TO_TEST]

    def _icp_for_candidate(self, candidate_points: np.ndarray):
        if self.template_points is None:
            return None

        centroid = candidate_points.mean(axis=0)
        r0 = pca_rotation(candidate_points)

        best_fit = 0.0
        best_rmse = float("inf")
        best_tf = None

        for ang in np.linspace(-np.pi, np.pi, 16, endpoint=False).tolist():
            r_init = r0 @ rot_z(ang)
            init = np.eye(4)
            init[:3, :3] = r_init
            init[:3, 3] = centroid

            source = o3d.geometry.PointCloud()
            source.points = o3d.utility.Vector3dVector(self.template_points)
            target = o3d.geometry.PointCloud()
            target.points = o3d.utility.Vector3dVector(candidate_points)

            result = o3d.pipelines.registration.registration_icp(
                source,
                target,
                ICP_THRESHOLD,
                init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_MAX_ITER),
            )

            if (result.fitness > best_fit) or (
                abs(result.fitness - best_fit) < 1e-6 and result.inlier_rmse < best_rmse
            ):
                best_fit = float(result.fitness)
                best_rmse = float(result.inlier_rmse)
                best_tf = result.transformation.copy()

        if best_tf is None:
            return None
        return best_tf, best_fit, best_rmse

    def _match_roi_points(self, roi_points: np.ndarray):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(roi_points)
        pcd = pcd.voxel_down_sample(VOXEL_SIZE_SCENE)
        if len(pcd.points) < MIN_CLUSTER_POINTS:
            self.get_logger().info(
                f"ICP Diagnose: zu wenige ROI-Punkte nach Voxel ({len(pcd.points)} < {MIN_CLUSTER_POINTS})"
            )
            return None, 0.0, float("inf")

        pcd_for_match = pcd
        try:
            _, inliers = pcd.segment_plane(
                distance_threshold=PLANE_DISTANCE_THRESHOLD,
                ransac_n=3,
                num_iterations=PLANE_RANSAC_ITER,
            )
            non_plane = pcd.select_by_index(inliers, invert=True)
            if len(non_plane.points) >= PLANE_MIN_KEEP_POINTS:
                pcd_for_match = non_plane
        except Exception:
            pass

        candidates = self._cluster_candidates(pcd_for_match)
        best_tf = None
        best_fit = 0.0
        best_rmse = float("inf")
        reject_small = 0
        reject_point_ratio = 0
        reject_extent = 0
        icp_attempts = 0

        for cand in candidates:
            cand_points = np.asarray(cand.points, dtype=np.float64)
            if len(cand_points) < MIN_CLUSTER_POINTS:
                reject_small += 1
                continue

            point_ratio = len(cand_points) / float(max(1, self.template_point_count))
            if point_ratio < POINT_COUNT_RATIO_MIN or point_ratio > POINT_COUNT_RATIO_MAX:
                reject_point_ratio += 1
                continue

            cand_extent = self._pca_extent(cand_points)
            extent_ratio = cand_extent / self.template_pca_extent
            if np.any(extent_ratio < PCA_EXTENT_RATIO_MIN) or np.any(extent_ratio > PCA_EXTENT_RATIO_MAX):
                reject_extent += 1
                continue

            icp_attempts += 1
            result = self._icp_for_candidate(cand_points)
            if result is None:
                continue

            tf_mat, fit, rmse = result
            if (fit > best_fit) or (abs(fit - best_fit) < 1e-6 and rmse < best_rmse):
                best_fit = fit
                best_rmse = rmse
                best_tf = tf_mat

        if best_tf is None and len(candidates) > 0:
            for cand in candidates:
                cand_points = np.asarray(cand.points, dtype=np.float64)
                if len(cand_points) < MIN_CLUSTER_POINTS:
                    continue
                icp_attempts += 1
                result = self._icp_for_candidate(cand_points)
                if result is None:
                    continue
                tf_mat, fit, rmse = result
                if (fit > best_fit) or (abs(fit - best_fit) < 1e-6 and rmse < best_rmse):
                    best_fit = fit
                    best_rmse = rmse
                    best_tf = tf_mat

            self.get_logger().info(
                "ICP Diagnose: Fallback ohne Shape-Filter aktiv "
                f"(Kandidaten={len(candidates)}, ICP-Versuche={icp_attempts})"
            )

        self.get_logger().info(
            "ICP Diagnose: "
            f"roi_raw={len(roi_points)}, roi_ds={len(pcd.points)}, match_pts={len(pcd_for_match.points)}, "
            f"kandidaten={len(candidates)}, icp_versuche={icp_attempts}, "
            f"verworfen_small={reject_small}, verworfen_ratio={reject_point_ratio}, verworfen_extent={reject_extent}"
        )

        if best_tf is None:
            return None, best_fit, best_rmse

        if best_fit < MIN_SEARCH_FITNESS or best_rmse > MAX_SEARCH_RMSE:
            return None, best_fit, best_rmse

        rot = best_tf[:3, :3]
        trans = best_tf[:3, 3]
        return (trans, rot, best_fit, best_rmse, best_tf), best_fit, best_rmse

    def _crop_roi_points_with_live_marker(self, points: np.ndarray):
        if self.crop_bounds is None:
            return None

        with self.lock:
            marker_pos = None if self.marker_position is None else self.marker_position.copy()
            marker_rot = None if self.marker_rotation is None else self.marker_rotation.copy()

        if marker_pos is None or marker_rot is None:
            return None

        x_min, x_max, y_min, y_max, z_min, z_max = self.crop_bounds
        points_rel = points - marker_pos
        points_marker = points_rel @ marker_rot.T

        mask = (
            (points_marker[:, 0] >= x_min)
            & (points_marker[:, 0] <= x_max)
            & (points_marker[:, 1] >= y_min)
            & (points_marker[:, 1] <= y_max)
            & (points_marker[:, 2] >= z_min)
            & (points_marker[:, 2] <= z_max)
        )

        roi = points[mask]
        if len(roi) < MIN_CLUSTER_POINTS:
            return None
        return roi, marker_pos, marker_rot

    def _scan_tool_once(self, template_id: str) -> PoseStamped:
        if self.template_points is None or self.template_pca_extent is None:
            raise RuntimeError("Template nicht fuer Scan geladen")

        self.get_logger().info("═══ TOOL SCAN START (integriert) ═══")
        deadline = time.time() + TOOL_SCAN_TIMEOUT_SEC
        min_stamp = self.get_clock().now().nanoseconds * 1e-9 + MIN_CLOUD_STAMP_AFTER_SCAN_SEC
        last_process = 0.0

        with self.lock:
            self.latest_live_pointcloud = None

        while rclpy.ok() and not self._shutdown:
            now = time.time()
            if now > deadline:
                raise RuntimeError(f"Timeout nach {TOOL_SCAN_TIMEOUT_SEC:.1f}s: Keine gueltige Zangenpose gefunden")

            if now - last_process < MATCH_INTERVAL_SEC:
                time.sleep(0.005)
                continue

            with self.lock:
                cloud_msg = self.latest_live_pointcloud
                self.latest_live_pointcloud = None

            if cloud_msg is None:
                time.sleep(0.005)
                continue

            cloud_stamp_sec = stamp_to_seconds(cloud_msg.header.stamp)
            if cloud_stamp_sec > 0.0 and cloud_stamp_sec < min_stamp:
                time.sleep(0.002)
                continue

            last_process = now

            pts = [
                [p[0], p[1], p[2]]
                for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            ]
            if len(pts) < MIN_CLUSTER_POINTS:
                continue

            points_np = np.asarray(pts, dtype=np.float64)
            if len(points_np) > MAX_RAW_POINTS:
                step = max(1, len(points_np) // MAX_RAW_POINTS)
                points_np = points_np[::step]

            crop_result = self._crop_roi_points_with_live_marker(points_np)
            if crop_result is None:
                continue
            roi_points, marker_pos, marker_rot = crop_result

            result, best_fit, best_rmse = self._match_roi_points(roi_points)
            if np.isfinite(best_rmse):
                self.get_logger().info(
                    f"ICP Bestwert im Scan: fitness={best_fit:.4f} (min {MIN_SEARCH_FITNESS:.2f}), "
                    f"rmse={best_rmse:.4f} (max {MAX_SEARCH_RMSE:.3f})"
                )
            else:
                self.get_logger().info(
                    f"ICP Bestwert im Scan: fitness={best_fit:.4f} (min {MIN_SEARCH_FITNESS:.2f}), rmse=n/a"
                )

            if result is None:
                continue

            translation, rotation, _, _, _ = result

            r_rel = marker_rot.T @ rotation
            p_rel = marker_rot.T @ (translation - marker_pos)
            qx, qy, qz, qw = rotmat_to_quat(r_rel)

            detected_pose = PoseStamped()
            detected_pose.header.stamp = self.get_clock().now().to_msg()
            detected_pose.header.frame_id = f"{self.frame_prefix}{self.target_marker_id}|{template_id}"
            detected_pose.pose.position.x = float(p_rel[0])
            detected_pose.pose.position.y = float(p_rel[1])
            detected_pose.pose.position.z = float(p_rel[2])
            detected_pose.pose.orientation.x = qx
            detected_pose.pose.orientation.y = qy
            detected_pose.pose.orientation.z = qz
            detected_pose.pose.orientation.w = qw

            self.pub_target_pose.publish(detected_pose)
            self.get_logger().info(
                f"✓ Zange erkannt und publiziert in {self.frame_prefix}{self.target_marker_id}: "
                f"x={p_rel[0]:.4f}, y={p_rel[1]:.4f}, z={p_rel[2]:.4f}, "
                f"qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}"
            )
            self.get_logger().info("═══ TOOL SCAN OK (integriert) ═══")
            return detected_pose

        raise RuntimeError("Node wird beendet")

    def _pointcloud2_to_xyz_and_records(self, cloud_msg: PointCloud2):
        try:
            if not cloud_msg.data or len(cloud_msg.data) == 0:
                return None, None

            x_offset = y_offset = z_offset = None
            for field in cloud_msg.fields:
                if field.name == "x":
                    x_offset = field.offset
                elif field.name == "y":
                    y_offset = field.offset
                elif field.name == "z":
                    z_offset = field.offset

            if x_offset is None or y_offset is None or z_offset is None:
                self.get_logger().error("PointCloud2 hat keine x/y/z Felder.")
                return None, None

            data_stream = io.BytesIO(bytes(cloud_msg.data))
            points = []
            records = []
            endian = ">" if cloud_msg.is_bigendian else "<"

            for i in range(cloud_msg.width * cloud_msg.height):
                base = i * cloud_msg.point_step
                data_stream.seek(base)
                record = data_stream.read(cloud_msg.point_step)
                if len(record) < cloud_msg.point_step:
                    continue

                try:
                    x = struct.unpack_from(f"{endian}f", record, x_offset)[0]
                    y = struct.unpack_from(f"{endian}f", record, y_offset)[0]
                    z = struct.unpack_from(f"{endian}f", record, z_offset)[0]

                    if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                        points.append([x, y, z])
                        records.append(record)
                except struct.error:
                    continue

            if len(points) == 0:
                return None, None

            return np.array(points, dtype=np.float32), records

        except Exception as exc:
            self.get_logger().error(f"Conversion Error: {exc}")
            return None, None

    def _records_to_pointcloud2(self, records, template_msg: PointCloud2) -> PointCloud2:
        try:
            cloud_msg = PointCloud2()
            cloud_msg.header = template_msg.header
            cloud_msg.height = 1
            cloud_msg.width = len(records)
            cloud_msg.is_dense = False
            cloud_msg.is_bigendian = template_msg.is_bigendian
            cloud_msg.point_step = template_msg.point_step
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.fields = template_msg.fields
            cloud_msg.data = b"".join(records)
            return cloud_msg

        except Exception as exc:
            self.get_logger().error(f"PointCloud2 Creation Error: {exc}")
            return None

    def _pointcloud2_to_xyz_rgb(self, cloud_msg):
        field_names = [f.name for f in cloud_msg.fields]
        has_rgb = "rgb" in field_names

        xyz = []
        rgb = []

        if has_rgb:
            iterator = pc2.read_points(
                cloud_msg,
                field_names=("x", "y", "z", "rgb"),
                skip_nans=True,
            )
            for p in iterator:
                x, y, z, rgb_float = p
                xyz.append([x, y, z])
                rgb.append(self._decode_rgb_float(rgb_float))
        else:
            iterator = pc2.read_points(
                cloud_msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            for p in iterator:
                x, y, z = p
                xyz.append([x, y, z])

        if len(xyz) == 0:
            return None, None

        xyz_np = np.array(xyz, dtype=np.float64)
        rgb_np = np.array(rgb, dtype=np.float64) if has_rgb and len(rgb) == len(xyz) else None
        return xyz_np, rgb_np

    @staticmethod
    def _decode_rgb_float(rgb_float):
        rgb_int = struct.unpack("I", struct.pack("f", float(rgb_float)))[0]
        r = ((rgb_int >> 16) & 0xFF) / 255.0
        g = ((rgb_int >> 8) & 0xFF) / 255.0
        b = (rgb_int & 0xFF) / 255.0
        return [r, g, b]

    @staticmethod
    def _quat_from_rotmat(r):
        return rotmat_to_quat(r)

    def _on_camera_info(self, msg: CameraInfo):
        if self.k is not None:
            return
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id or self.camera_frame
        self.get_logger().info(f"Kamera-Info erhalten. Frame: {self.camera_frame}")

    def _image_to_bgr8(self, msg: Image):
        if msg.height == 0 or msg.width == 0:
            return None

        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        expected_rgb = msg.height * msg.width * 3

        if msg.encoding in ("bgr8", "rgb8"):
            if data.size < expected_rgb:
                return None
            img = data[:expected_rgb].reshape((msg.height, msg.width, 3))
            if msg.encoding == "rgb8":
                img = img[:, :, ::-1]
            return np.ascontiguousarray(img)

        if msg.encoding == "mono8":
            expected_mono = msg.height * msg.width
            if data.size < expected_mono:
                return None
            mono = data[:expected_mono].reshape((msg.height, msg.width))
            bgr = np.repeat(mono[:, :, None], 3, axis=2)
            return np.ascontiguousarray(bgr)

        return None

    def _on_image(self, msg: Image):
        if self.k is None:
            return

        frame = self._image_to_bgr8(msg)
        if frame is None:
            return

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

            if int(marker_id) == self.target_marker_id:
                with self.lock:
                    self.marker_position = tvec[0].copy()
                    self.marker_rotation = rmat.copy()

    def _on_pointcloud(self, msg: PointCloud2):
        with self.lock:
            if not self._got_first_cloud:
                self._got_first_cloud = True
                self.get_logger().info("Erste Point Cloud empfangen.")

            self.latest_live_pointcloud = msg

            if self.snapshot_pointcloud is None:
                self.current_pointcloud = msg

    def _publish_outputs_timer(self):
        with self.lock:
            if self.snapshot_pointcloud is None:
                if self.current_pointcloud is not None:
                    self.pub_full_cloud.publish(self.current_pointcloud)
                return

            self.pub_full_cloud.publish(self.snapshot_pointcloud)

            if self.cached_cropped_pointcloud is not None:
                self.pub_cropped_cloud.publish(self.cached_cropped_pointcloud)

            if self.snapshot_marker_position is not None and self.snapshot_marker_rotation is not None:
                axes_marker = self._build_snapshot_axes_markers_locked(self.snapshot_pointcloud.header)
                if axes_marker is not None:
                    self.pub_axes_marker.publish(axes_marker)

    def _build_snapshot_axes_markers_locked(self, header):
        if self.snapshot_marker_position is None or self.snapshot_marker_rotation is None:
            return None

        origin = np.array(self.snapshot_marker_position, dtype=np.float64)
        rmat = np.array(self.snapshot_marker_rotation, dtype=np.float64)
        axis_len = float(max(self.marker_size * 0.8, 0.03))
        shaft_diameter = float(max(self.marker_size * 0.08, 0.003))
        head_diameter = shaft_diameter * 1.8
        head_length = axis_len * 0.2

        marker_array = MarkerArray()
        axis_specs = [
            (0, (1.0, 0.0, 0.0, 1.0)),
            (1, (0.0, 1.0, 0.0, 1.0)),
            (2, (0.0, 0.0, 1.0, 1.0)),
        ]

        for axis_index, rgba in axis_specs:
            direction = rmat[:, axis_index]
            end = origin + axis_len * direction

            m = Marker()
            m.header = header
            m.ns = "snapshot_axes"
            m.id = axis_index
            m.type = Marker.ARROW
            m.action = Marker.ADD

            p0 = Point()
            p0.x, p0.y, p0.z = float(origin[0]), float(origin[1]), float(origin[2])
            p1 = Point()
            p1.x, p1.y, p1.z = float(end[0]), float(end[1]), float(end[2])
            m.points = [p0, p1]

            m.scale.x = shaft_diameter
            m.scale.y = head_diameter
            m.scale.z = head_length
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0

            marker_array.markers.append(m)

        return marker_array

    def _publish_crop_preview_locked(self):
        if self.snapshot_pointcloud is None or not self.crop_bounds:
            return
        if self.snapshot_marker_position is None or self.snapshot_marker_rotation is None:
            self.get_logger().warn("Keine Snapshot-Marker-Pose verfuegbar, Crop-Vorschau nicht moeglich.")
            return

        self.cached_cropped_pointcloud = self._crop_pointcloud(self.snapshot_pointcloud)
        if self.cached_cropped_pointcloud is not None:
            self.pub_cropped_cloud.publish(self.cached_cropped_pointcloud)
        else:
            self.get_logger().warn("Crop berechnet: 0 Punkte im Bereich.")

    def _crop_pointcloud(self, cloud_msg: PointCloud2) -> PointCloud2:
        if self.snapshot_marker_position is None or self.snapshot_marker_rotation is None or self.crop_bounds is None:
            return None

        try:
            points, point_records = self._pointcloud2_to_xyz_and_records(cloud_msg)
            if points is None or len(points) == 0:
                return None

            x_min, x_max, y_min, y_max, z_min, z_max = self.crop_bounds
            points_relative = points - self.snapshot_marker_position
            points_in_marker_frame = points_relative @ self.snapshot_marker_rotation.T

            mask = (
                (points_in_marker_frame[:, 0] >= x_min)
                & (points_in_marker_frame[:, 0] <= x_max)
                & (points_in_marker_frame[:, 1] >= y_min)
                & (points_in_marker_frame[:, 1] <= y_max)
                & (points_in_marker_frame[:, 2] >= z_min)
                & (points_in_marker_frame[:, 2] <= z_max)
            )

            cropped_points = points[mask]
            cropped_records = [point_records[i] for i in np.where(mask)[0]]

            if len(cropped_points) == 0:
                self.get_logger().warn("Keine Punkte im definierten Bereich!")
                return None

            cropped_msg = self._records_to_pointcloud2(cropped_records, cloud_msg)
            return cropped_msg

        except Exception as exc:
            self.get_logger().error(f"Cropping-Fehler: {exc}")
            return None

    def _save_cropped_snapshot_locked(self, file_name):
        cloud_msg = self.cached_cropped_pointcloud
        if cloud_msg is None:
            return None

        if not file_name.endswith(".pcd"):
            file_name = f"{file_name}.pcd"

        pcd_path = os.path.join(self.snapshot_save_dir, file_name)
        xyz, rgb = self._pointcloud2_to_xyz_rgb(cloud_msg)
        if xyz is None or len(xyz) == 0:
            self.get_logger().warn("Keine gueltigen Punkte zum Speichern vorhanden.")
            return None

        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            if rgb is not None and len(rgb) == len(xyz):
                pcd.colors = o3d.utility.Vector3dVector(rgb)

            ok = o3d.io.write_point_cloud(pcd_path, pcd)
            if not ok:
                raise RuntimeError("Open3D write_point_cloud lieferte False")
            return pcd_path

        except Exception as exc:
            self.get_logger().warn(f"Open3D Fehler: {exc}. Fallback auf NPY.")
            npy_path = pcd_path.replace(".pcd", ".npy")
            np.save(npy_path, xyz)
            return npy_path

    def _workflow(self) -> None:
        try:
            self.get_logger().info("=== Schritt 1: Snapshot und Crop ===")
            print("\n" + "=" * 70)
            print("SCHRITT 1: Snapshot")
            print("-" * 70)
            print("Wenn die Point Cloud und der ArUco Marker stabil sichtbar sind, ENTER druecken.")

            while rclpy.ok() and not self._shutdown:
                with self.lock:
                    ready = self.current_pointcloud is not None and self.marker_position is not None and self.marker_rotation is not None
                if ready:
                    break
                time.sleep(0.2)

            _ = input("Snapshot jetzt aufnehmen? ENTER fuer Aufnahme, sonst Ctrl+C: ")

            with self.lock:
                if self.current_pointcloud is None:
                    raise RuntimeError("Noch keine Point Cloud empfangen")
                if self.marker_position is None or self.marker_rotation is None:
                    raise RuntimeError("Noch keine Marker-Pose verfuegbar")

                self.snapshot_pointcloud = copy.deepcopy(self.current_pointcloud)
                self.snapshot_marker_position = self.marker_position.copy()
                self.snapshot_marker_rotation = self.marker_rotation.copy()
                self.cached_cropped_pointcloud = None
                self._snapshot_taken = True

            print("\nStandaufnahme erstellt.")
            print("Crop-Format: x_min x_max y_min y_max z_min z_max")
            print(f"Default: {self.crop_bounds_default}")
            crop_input = input("Crop-Grenzen eingeben oder Enter fuer Default: ").strip()
            crop_text = crop_input or self.crop_bounds_default
            parts = crop_text.split()
            if len(parts) != 6:
                raise RuntimeError("Crop-Grenzen muessen 6 Zahlen enthalten")

            self.crop_bounds = tuple(float(p) for p in parts)
            with self.lock:
                self._publish_crop_preview_locked()

            default_snapshot_name = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            snapshot_name = input(f"Snapshot-Dateiname ohne Endung (Enter={default_snapshot_name}): ").strip() or default_snapshot_name
            with self.lock:
                saved_snapshot_path = self._save_cropped_snapshot_locked(snapshot_name)

            if saved_snapshot_path is None:
                raise RuntimeError("Snapshot konnte nicht gespeichert werden")
            self.get_logger().info(f"Snapshot gespeichert: {saved_snapshot_path}")

            self.get_logger().info("=== Schritt 2: Preprocess ===")
            raw_pcd = o3d.io.read_point_cloud(saved_snapshot_path)
            processed_pcd, centroid = preprocess_template_cloud(
                raw_pcd,
                ransac_threshold=self.ransac_threshold,
                ransac_iterations=self.ransac_iterations,
                cluster_eps=self.cluster_eps,
                cluster_min_points=self.cluster_min_points,
                outlier_neighbors=self.outlier_neighbors,
                outlier_std=self.outlier_std,
                voxel_size=self.voxel_size,
                downsample=True,
            )

            template_name = f"{safe_slug(snapshot_name)}_clean_direction.pcd"
            template_path = os.path.join(self.template_save_dir, template_name)
            self._save_pointcloud(template_path, processed_pcd)

            self.get_logger().info(f"Template gespeichert: {template_path}")
            self.get_logger().info(
                "Template-Centroid (vor Rotation): "
                f"[{centroid[0]:+.6f}, {centroid[1]:+.6f}, {centroid[2]:+.6f}]"
            )
            self._load_template_for_scan(template_path)

            meta_path = os.path.join(self.template_save_dir, f"{safe_slug(snapshot_name)}_pipeline.yaml")
            self._save_text_file(
                meta_path,
                self._format_yaml_text(
                    template_id=safe_slug(snapshot_name),
                    snapshot_pcd=saved_snapshot_path,
                    template_pcd=template_path,
                    crop_bounds=self.crop_bounds,
                    offset_translation=np.array([0.0, 0.0, 0.0], dtype=np.float64),
                    offset_quat=(0.0, 0.0, 0.0, 1.0),
                ),
            )
            self.get_logger().info(f"Pipeline-Metadaten gespeichert: {meta_path}")

            self.get_logger().info("=== Schritt 3: Gripper-Offset Kalibrierung ===")
            print("\n" + "=" * 70)
            print("SCHRITT 3: Offset-Kalibrierung")
            print("-" * 70)
            print("Integrierter Template-Scan mit dem gerade erzeugten Template.")
            print("Danach Roboter manuell in die echte Greifpose fahren und ENTER druecken.")

            input("\nEnter druecken, um den integrierten Zangen-Scan zu starten...")
            template_id = safe_slug(snapshot_name)
            detected_pose_raw = self._scan_tool_once(template_id)
            detected_base_p, detected_base_q = self._target_in_base(detected_pose_raw)

            detected_base = PoseStamped()
            detected_base.header.frame_id = self.base
            detected_base.header.stamp = self.get_clock().now().to_msg()
            detected_base.pose.position.x = float(detected_base_p[0])
            detected_base.pose.position.y = float(detected_base_p[1])
            detected_base.pose.position.z = float(detected_base_p[2])
            detected_base.pose.orientation = detected_base_q

            print("\n--- ERKANNTE POSE ---")
            print(self._format_pose("detected_raw", detected_pose_raw))
            print(self._format_pose("detected_base", detected_base))
            print(f"template_id: {template_id}")

            input("\nRoboter manuell in echte Greifpose fahren, dann Enter druecken...")
            manual_tcp = self._wait_for_tcp_pose("manual_tcp")

            print("\n--- MANUELLE GREIFPOSE (TCP) ---")
            print(self._format_pose("manual_tcp", manual_tcp))

            p_off, q_off, rpy_off = self._compute_offset(detected_base, manual_tcp)

            print("\n--- BERECHNETER OFFSET (detected_center -> grasp_tcp) ---")
            print(
                "translation_xyz_m = "
                f"({p_off[0]:+.6f}, {p_off[1]:+.6f}, {p_off[2]:+.6f})"
            )
            print(
                "rotation_quat_xyzw = "
                f"({q_off[0]:+.6f}, {q_off[1]:+.6f}, {q_off[2]:+.6f}, {q_off[3]:+.6f})"
            )
            print(
                "rotation_rpy_rad = "
                f"({rpy_off[0]:+.6f}, {rpy_off[1]:+.6f}, {rpy_off[2]:+.6f})"
            )

            final_yaml_path = os.path.join(self.offset_save_dir, f"{safe_slug(template_id)}_grasp_offset.yaml")
            self._save_text_file(
                final_yaml_path,
                self._format_yaml_text(
                    template_id=template_id,
                    snapshot_pcd=saved_snapshot_path,
                    template_pcd=template_path,
                    crop_bounds=self.crop_bounds,
                    offset_translation=p_off,
                    offset_quat=q_off,
                ),
            )
            self.get_logger().info(f"Offset-YAML gespeichert: {final_yaml_path}")

            print("\nCopy-paste fuer TEMPLATE_GRASP_OFFSETS:")
            print(
                f'"{template_id}": {{"translation_xyz_m": '
                f'({p_off[0]:+.6f}, {p_off[1]:+.6f}, {p_off[2]:+.6f}), '
                f'"rotation_quat_xyzw": '
                f'({q_off[0]:+.6f}, {q_off[1]:+.6f}, {q_off[2]:+.6f}, {q_off[3]:+.6f})}},'
            )

            self.get_logger().info("Workflow abgeschlossen")
            self._shutdown = True
            rclpy.shutdown()

        except (EOFError, KeyboardInterrupt):
            self.get_logger().info("Workflow durch Benutzer beendet")
            self._shutdown = True
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            self.get_logger().error(f"Workflow fehlgeschlagen: {exc}")
            self._shutdown = True
            if rclpy.ok():
                rclpy.shutdown()

    def destroy_node(self):
        self._shutdown = True
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TemplateScanGraspPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
