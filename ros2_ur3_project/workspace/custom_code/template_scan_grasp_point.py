#!/usr/bin/env python3
"""
Template Scan + Grasp Point workflow.

Eigener monolithischer Orchestrator fuer:
1) Homefahrt in die Kameraposition
2) ArUco-Snapshot + Cropping + Speicherung als PCD
3) Offline-Preprocessing zum Template
4) Gripper-Offset-Kalibrierung mit manueller Greifpose

Die Datei importiert keine Funktionen aus den anderen Projekt-Skripten,
sondern enthaelt die benoetigte Logik lokal.
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
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningSceneComponents,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


TARGET_TOPIC = "/tool_target_pose"
DEFAULT_TEMPLATE_ID = "cropv1_clean_direction"

SERVICE_TIMEOUT_SEC = 8.0
ACTION_TIMEOUT_SEC = 30.0
IK_SERVICE_TIMEOUT_SEC = 20.0
PLAN_SERVICE_TIMEOUT_SEC = 25.0
JOINT_STATE_WAIT_SEC = 10.0

HOVER_Z_OFFSET_M = 0.02
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

HOME_TABLE_X_M = 0.17
HOME_TABLE_Y_M = -0.04
HOME_TABLE_Z_M = 0.14
HOME_YAW_RAD = math.pi

ARUCO_IN_BASE_X_M = 0.15
ARUCO_IN_BASE_Y_M = 0.15
ARUCO_IN_BASE_Z_M = 0.0
ARUCO_IN_BASE_RX = 0.0
ARUCO_IN_BASE_RY = 0.0
ARUCO_IN_BASE_RZ = 0.0

DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class WorkflowState(Enum):
    MOVE_HOME = 1
    SNAPSHOT = 2
    PREPROCESS = 3
    CALIBRATE = 4
    DONE = 5


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
        super().__init__("template_scan_grasp_point")

        script_dir = os.path.dirname(os.path.abspath(__file__))

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("pointcloud_topic", "/camera/camera/depth/color/points")
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
        self.declare_parameter("return_home_after_grasp", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
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
        self.return_home_after_grasp = bool(self.get_parameter("return_home_after_grasp").value)

        if dict_name not in DICT_MAP:
            self.get_logger().warn(f"Unbekanntes Dictionary {dict_name}, nutze DICT_4X4_50")
            dict_name = "DICT_4X4_50"

        os.makedirs(self.snapshot_save_dir, exist_ok=True)
        os.makedirs(self.template_save_dir, exist_ok=True)
        os.makedirs(self.offset_save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[dict_name])
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        self.group = "ur_manipulator"
        self.ee_link = "tcp"
        self.base = "base_link"
        self.action_name = "/scaled_joint_trajectory_controller/follow_joint_trajectory"

        self.cli_ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cli_plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cli_fk = self.create_client(GetPositionFK, "/compute_fk")
        self.cli_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.exec_ac = ActionClient(self, FollowJointTrajectory, self.action_name)

        for cli in [self.cli_ik, self.cli_plan, self.cli_fk, self.cli_scene]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("MoveIt Service nicht verfuegbar")
        if not self.exec_ac.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Trajectory Action Server nicht verfuegbar")

        self.k = None
        self.dist = None
        self.camera_frame = "camera_color_optical_frame"

        self.latest_joint_state: Optional[JointState] = None
        self.js_lock = threading.Lock()

        self.latest_target_pose: Optional[PoseStamped] = None
        self.latest_template_id = DEFAULT_TEMPLATE_ID
        self.target_lock = threading.Lock()
        self.target_event = threading.Event()

        self.current_pointcloud: Optional[PointCloud2] = None
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

        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, qos_pose)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_pose)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self._on_pointcloud, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, TARGET_TOPIC, self._on_target_pose, qos_pose)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)

        self.pub_cropped_cloud = self.create_publisher(PointCloud2, "/cloud_cropped", 10)
        self.pub_full_cloud = self.create_publisher(PointCloud2, "/cloud_full", 10)
        self.pub_axes_marker = self.create_publisher(MarkerArray, "/snapshot_axes", 10)

        self.publish_timer = self.create_timer(1.0, self._publish_outputs_timer)

        self.workflow_thread = threading.Thread(target=self._workflow, daemon=True)
        self.workflow_thread.start()

        self.get_logger().info("Template Scan + Grasp Point gestartet")
        self.get_logger().info(f"  Image: {self.image_topic}")
        self.get_logger().info(f"  PointCloud: {self.pointcloud_topic}")
        self.get_logger().info(f"  Target topic: {TARGET_TOPIC}")
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

    def _on_joint_state(self, msg: JointState) -> None:
        with self.js_lock:
            self.latest_joint_state = msg

    def _on_target_pose(self, msg: PoseStamped) -> None:
        frame, template_id = split_frame_and_template(msg.header.frame_id)
        msg.header.frame_id = frame
        with self.target_lock:
            self.latest_target_pose = msg
            self.latest_template_id = template_id
        self.target_event.set()

    def get_robot_state(self) -> RobotState:
        try:
            req = GetPlanningScene.Request()
            req.components = PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE)
            fut = self.cli_scene.call_async(req)
            res = self._wait_future_result(fut, "GetPlanningScene", SERVICE_TIMEOUT_SEC)
            if res is not None and res.scene is not None:
                return res.scene.robot_state
        except Exception as exc:
            self.get_logger().warn(f"[STATE] GetPlanningScene fehlgeschlagen, nutze /joint_states Fallback: {exc}")

        with self.js_lock:
            js = self.latest_joint_state

        if js is None or not js.name:
            raise RuntimeError("Noch keine /joint_states erhalten")

        rs = RobotState()
        rs.joint_state.name = list(js.name)
        rs.joint_state.position = list(js.position)
        rs.joint_state.velocity = list(js.velocity)
        rs.joint_state.effort = list(js.effort)
        rs.joint_state.header = js.header
        return rs

    def plan_to_pose_quat(self, x: float, y: float, z: float, q: Quaternion) -> JointTrajectory:
        pose = PoseStamped()
        pose.header.frame_id = self.base
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation = q

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = self.group
        ik_req.ik_request.robot_state = self.get_robot_state()
        ik_req.ik_request.pose_stamped = pose
        ik_req.ik_request.ik_link_name = self.ee_link
        ik_req.ik_request.timeout = Duration(sec=2, nanosec=0)

        ik_res = None
        last_ik_error = None
        for avoid_collisions in (True, False):
            ik_req.ik_request.avoid_collisions = avoid_collisions
            fut = self.cli_ik.call_async(ik_req)
            try:
                candidate = self._wait_future_result(fut, "ComputeIK", IK_SERVICE_TIMEOUT_SEC)
            except Exception as exc:
                last_ik_error = exc
                continue

            if candidate is None:
                last_ik_error = RuntimeError("ComputeIK lieferte keine Antwort")
                continue

            if candidate.error_code.val == candidate.error_code.SUCCESS:
                ik_res = candidate
                break

            last_ik_error = RuntimeError(f"IK error_code={candidate.error_code.val}")

        if ik_res is None:
            raise RuntimeError(f"Kein IK-Ergebnis: {last_ik_error}")

        constraints = Constraints()
        for name, pos in zip(ik_res.solution.joint_state.name, ik_res.solution.joint_state.position):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 1e-3
            jc.tolerance_below = 1e-3
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        mpr = MotionPlanRequest()
        mpr.group_name = self.group
        mpr.goal_constraints = [constraints]
        mpr.start_state = self.get_robot_state()
        mpr.max_velocity_scaling_factor = 0.3
        mpr.max_acceleration_scaling_factor = 0.3
        mpr.allowed_planning_time = 5.0
        mpr.num_planning_attempts = 3

        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = mpr
        fut = self.cli_plan.call_async(plan_req)
        plan_res = self._wait_future_result(fut, "GetMotionPlan", PLAN_SERVICE_TIMEOUT_SEC)
        if plan_res is None or plan_res.motion_plan_response is None:
            raise RuntimeError("GetMotionPlan lieferte keine Antwort")

        jt = plan_res.motion_plan_response.trajectory.joint_trajectory
        if not jt.points:
            raise RuntimeError("Keine Trajektorie gefunden")
        return jt

    def execute_trajectory(self, jt: JointTrajectory) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        fut = self.exec_ac.send_goal_async(goal)
        gh = self._wait_future_result(fut, "SendGoal", ACTION_TIMEOUT_SEC)
        if gh is None or not gh.accepted:
            raise RuntimeError("Goal abgelehnt")

        res_fut = gh.get_result_async()
        result = self._wait_future_result(res_fut, "GetResult", ACTION_TIMEOUT_SEC)
        if result is None or getattr(result.result, "error_code", 0) != 0:
            raise RuntimeError("Ausfuehrung fehlgeschlagen")

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

    def _move_camera_pose(self) -> None:
        q_home = rpy_to_quat(
            GRIPPER_FIXED_ROLL,
            GRIPPER_FIXED_PITCH,
            HOME_YAW_RAD + TOOL_YAW_OFFSET,
        )
        home_x = TABLE_ORIGIN_IN_BASE_X_M + (TABLE_TO_BASE_X_SIGN * HOME_TABLE_X_M)
        home_y = TABLE_ORIGIN_IN_BASE_Y_M + (TABLE_TO_BASE_Y_SIGN * HOME_TABLE_Y_M)
        home_z = TABLE_ORIGIN_IN_BASE_Z_M + (TABLE_TO_BASE_Z_SIGN * HOME_TABLE_Z_M)

        jt_home = self.plan_to_pose_quat(home_x, home_y, home_z, q_home)
        self.execute_trajectory(jt_home)

    def _get_current_tcp_pose(self) -> PoseStamped:
        req = GetPositionFK.Request()
        req.header.frame_id = self.base
        req.fk_link_names = [self.ee_link]
        req.robot_state = self.get_robot_state()

        fut = self.cli_fk.call_async(req)
        res = self._wait_future_result(fut, "GetPositionFK", SERVICE_TIMEOUT_SEC)
        if res is None or not res.pose_stamped:
            raise RuntimeError("FK lieferte keine Pose")
        return res.pose_stamped[0]

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

    def _on_image(self, msg: Image):
        if self.k is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
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

            if self.snapshot_pointcloud is None:
                self.current_pointcloud = msg

    def _publish_outputs_timer(self):
        with self.lock:
            if self.snapshot_pointcloud is None:
                if self.current_pointcloud is not None:
                    self.pub_full_cloud.publish(self.current_pointcloud)
                return

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
            self.get_logger().info("=== Schritt 1: Homefahrt zur Kameraposition ===")
            self._move_camera_pose()
            self.get_logger().info("Homeposition erreicht")

            self.get_logger().info("=== Schritt 2: Snapshot und Crop ===")
            print("\n" + "=" * 70)
            print("SCHRITT 2: Snapshot")
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

            self.get_logger().info("=== Schritt 3: Preprocess ===")
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

            self.get_logger().info("=== Schritt 4: Gripper-Offset Kalibrierung ===")
            print("\n" + "=" * 70)
            print("SCHRITT 4: Offset-Kalibrierung")
            print("-" * 70)
            print(f"Warte auf Pose auf {TARGET_TOPIC} ...")
            print("Danach Roboter manuell in die echte Greifpose fahren und ENTER druecken.")

            detected_pose_raw, template_id = self._wait_for_target_pose()
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
            manual_tcp = self._get_current_tcp_pose()

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

            if self.return_home_after_grasp:
                try:
                    self.get_logger().info("Rueckfahrt in Kameraposition...")
                    self._move_camera_pose()
                except Exception as exc:
                    self.get_logger().warn(f"Rueckfahrt zur Kamera fehlgeschlagen: {exc}")

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
