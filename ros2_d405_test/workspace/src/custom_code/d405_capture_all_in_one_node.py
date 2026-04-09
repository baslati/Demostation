#!/usr/bin/env python3
"""
D405 All-in-One (Single File)

Eine einzige Python-Datei mit:
1) ROI-ICP Matcher Node (integriert)
2) Capture-to-Pose Node (Enter-bestaetigt)

Kein Import/Start anderer Python-Dateien noetig.
"""

import os
import threading
import time
from typing import Optional, Tuple

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPEN3D_CPU_THREAD_COUNT", "1")

import cv2
import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


# ===== Shared =====
TARGET_TOPIC = "/tool_target_pose"
STATUS_TOPIC = "/tool_detection_status"
MARKER_FRAME = "aruco_0"
DETECTED_FRAME = "detected_cropv1_center"
CAPTURE_TIMEOUT_SEC = 8.0


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

    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ], dtype=np.float64)


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


# ===== ROI-ICP Matcher (integrated) =====
POINTCLOUD_TOPIC = "/camera/camera/depth/color/points"
IMAGE_TOPIC = "/camera/camera/color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
ROI_CLOUD_TOPIC = "/cloud_roi_cropped"
TEMPLATE_PATH = "/workspace/src/custom_packages/custom_code/templates/cropv1_clean_direction.pcd"

MARKER_SIZE = 0.05
MARKER_ID = 0
FRAME_PREFIX = "aruco_"
CROP_BOUNDS_MARKER = (-0.03, 0.38, -0.22, 0.03, 0.008, 0.03)

MATCH_INTERVAL = 0.35
MAX_RAW_POINTS = 60000
MAX_ROI_CLOUD_POINTS = 30000
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
MAX_SEARCH_RMSE = 0.1

POSE_SMOOTH_ALPHA = 0.55
ARUCO_FILTER_ALPHA = 0.12
ARUCO_MAX_JUMP_M = 0.02
ARUCO_DEADBAND_M = 0.0015
TRACKING_MIN_FITNESS = 0.60
TRACKING_MAX_RMSE = 0.010
REDETECT_PERIOD = 2
PUBLISH_ROI_CLOUD = False

POINT_COUNT_RATIO_MIN = 0.45
POINT_COUNT_RATIO_MAX = 1.80
PCA_EXTENT_RATIO_MIN = 0.55
PCA_EXTENT_RATIO_MAX = 1.70

DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
}


def quat_from_rotmat(r):
    return rotmat_to_quat(r)


def pca_rotation(points):
    centered = points - points.mean(axis=0)
    cov = centered.T @ centered / max(1, len(points) - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    r = eigvecs[:, order]
    if np.linalg.det(r) < 0.0:
        r[:, 2] *= -1.0
    return r


def rot_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


class TemplateMatchingRoiIcpNode(Node):
    def __init__(self):
        super().__init__("template_matching_roi_icp_node_integrated")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.capture_enabled = False

        self.k = None
        self.dist = None
        self.camera_frame = "camera_color_optical_frame"
        self.marker_position = None
        self.marker_rotation = None
        self.marker_filtered_tvec = None
        self.marker_filtered_rvec = None
        self.marker_lock = threading.Lock()

        self.last_match_time = 0.0
        self.last_translation = None
        self.last_transform = None
        self.match_cycle = 0

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP["DICT_4X4_50"])
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        if not self._load_template():
            self.get_logger().error("Template konnte nicht geladen werden.")
            return

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )

        self.create_subscription(PointCloud2, POINTCLOUD_TOPIC, self._on_cloud, qos_sensor)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, qos_sensor)
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, qos_sensor)

        self.pub_roi_cloud = None
        self.roi_cloud_lock = threading.Lock()
        self.roi_cloud_queue = (None, None)
        if PUBLISH_ROI_CLOUD:
            self.pub_roi_cloud = self.create_publisher(PointCloud2, ROI_CLOUD_TOPIC, qos_pub)
            self.roi_publish_thread = threading.Thread(target=self._roi_publish_worker, daemon=True)
            self.roi_publish_thread.start()

        self.get_logger().info("Integrierter ROI-ICP Matcher bereit (wartet auf Capture-Freigabe)")

    def set_capture_enabled(self, enabled: bool) -> None:
        self.capture_enabled = enabled
        if enabled:
            self.last_match_time = 0.0
            self.last_transform = None
            self.last_translation = None
            self.match_cycle = 0
            self.get_logger().info("Matcher aktiviert fuer aktuellen Capture")

    def _load_template(self):
        if not os.path.exists(TEMPLATE_PATH):
            self.get_logger().error(f"Template fehlt: {TEMPLATE_PATH}")
            return False
        pcd = o3d.io.read_point_cloud(TEMPLATE_PATH)
        if len(pcd.points) < 50:
            self.get_logger().error("Template ist leer oder zu klein")
            return False

        pcd = pcd.voxel_down_sample(VOXEL_SIZE_TEMPLATE)
        pts = np.asarray(pcd.points, dtype=np.float64)
        centroid = pts.mean(axis=0)
        self.template_points = pts - centroid
        self.template_point_count = len(self.template_points)
        self.template_pca_extent = self._pca_extent(self.template_points)

        self.get_logger().info(f"Template geladen: {len(self.template_points)} Punkte (zentriert)")
        return True

    def _pca_extent(self, points):
        if len(points) < 10:
            return np.array([1e-6, 1e-6, 1e-6], dtype=np.float64)
        r = pca_rotation(points)
        centered = points - points.mean(axis=0)
        proj = centered @ r
        return np.maximum(np.ptp(proj, axis=0), 1e-6)

    def _on_camera_info(self, msg: CameraInfo):
        if self.k is not None:
            return
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id or self.camera_frame
        self.get_logger().info(f"CameraInfo erhalten: {self.camera_frame}")

    def _image_to_bgr8(self, msg: Image):
        if msg.height == 0 or msg.width == 0:
            return None
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ("bgr8", "rgb8"):
            expected = msg.height * msg.width * 3
            if data.size < expected:
                return None
            img = data[:expected].reshape((msg.height, msg.width, 3))
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img
        if msg.encoding == "mono8":
            expected = msg.height * msg.width
            if data.size < expected:
                return None
            mono = data[:expected].reshape((msg.height, msg.width))
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        return None

    def _on_image(self, msg: Image):
        # ArUco kontinuierlich laufen lassen, damit Marker-TF beim Capture sofort verfuegbar ist.
        if self.k is None:
            return

        frame = self._image_to_bgr8(msg)
        if frame is None:
            return

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, self.k, self.dist)
        for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
            if int(marker_id) != MARKER_ID:
                continue

            raw_rvec = np.array(rvec[0], dtype=np.float64).reshape(3)
            raw_tvec = np.array(tvec[0], dtype=np.float64).reshape(3)

            if self.marker_filtered_tvec is None:
                filt_tvec = raw_tvec
                filt_rvec = raw_rvec
            else:
                jump = np.linalg.norm(raw_tvec - self.marker_filtered_tvec)
                if jump > ARUCO_MAX_JUMP_M or jump < ARUCO_DEADBAND_M:
                    filt_tvec = self.marker_filtered_tvec
                    filt_rvec = self.marker_filtered_rvec
                else:
                    if np.dot(raw_rvec, self.marker_filtered_rvec) < 0.0:
                        raw_rvec = -raw_rvec
                    filt_tvec = ARUCO_FILTER_ALPHA * raw_tvec + (1.0 - ARUCO_FILTER_ALPHA) * self.marker_filtered_tvec
                    filt_rvec = ARUCO_FILTER_ALPHA * raw_rvec + (1.0 - ARUCO_FILTER_ALPHA) * self.marker_filtered_rvec

            self.marker_filtered_tvec = filt_tvec
            self.marker_filtered_rvec = filt_rvec

            r_mat, _ = cv2.Rodrigues(filt_rvec)
            qx, qy, qz, qw = quat_from_rotmat(r_mat)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.camera_frame
            tf_msg.child_frame_id = f"{FRAME_PREFIX}{int(marker_id)}"
            tf_msg.transform.translation.x = float(filt_tvec[0])
            tf_msg.transform.translation.y = float(filt_tvec[1])
            tf_msg.transform.translation.z = float(filt_tvec[2])
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

            with self.marker_lock:
                self.marker_position = np.array(filt_tvec, dtype=np.float64)
                self.marker_rotation = np.array(r_mat, dtype=np.float64)

    def _crop_roi(self, points):
        with self.marker_lock:
            marker_pos = None if self.marker_position is None else self.marker_position.copy()
            marker_rot = None if self.marker_rotation is None else self.marker_rotation.copy()
        if marker_pos is None or marker_rot is None:
            return None

        x_min, x_max, y_min, y_max, z_min, z_max = CROP_BOUNDS_MARKER
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
        return roi

    def _on_cloud(self, msg: PointCloud2):
        if not self.capture_enabled:
            return

        now = time.time()
        if now - self.last_match_time < MATCH_INTERVAL:
            return

        try:
            points = [[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
            if len(points) < MIN_CLUSTER_POINTS:
                return

            points_np = np.asarray(points, dtype=np.float64)
            if len(points_np) > MAX_RAW_POINTS:
                step = max(1, len(points_np) // MAX_RAW_POINTS)
                points_np = points_np[::step]

            roi_points = self._crop_roi(points_np)
            self.last_match_time = now
            if roi_points is None:
                return

            pose = self._match_roi_points(roi_points)
            if pose is None:
                return

            translation, rotation, fitness, rmse, transform = pose
            if self.last_translation is not None:
                translation = POSE_SMOOTH_ALPHA * translation + (1.0 - POSE_SMOOTH_ALPHA) * self.last_translation

            self.last_translation = translation
            self.last_transform = transform
            self._publish_detected_tf(translation, rotation, msg.header)
            self.get_logger().info(
                f"Match: Fitness={fitness:.3f} | RMSE={rmse:.4f} m | "
                f"Pos=({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})"
            )
        except Exception as e:
            self.get_logger().error(f"Matching-Fehler: {e}")

    def _roi_publish_worker(self):
        while True:
            time.sleep(0.05)
            with self.roi_cloud_lock:
                header, points_np = self.roi_cloud_queue
            if header is None or points_np is None or self.pub_roi_cloud is None:
                continue
            try:
                p = points_np
                if len(p) > MAX_ROI_CLOUD_POINTS:
                    step = max(1, len(p) // MAX_ROI_CLOUD_POINTS)
                    p = p[::step]
                msg = pc2.create_cloud_xyz32(header, p[:, :3])
                self.pub_roi_cloud.publish(msg)
            except Exception:
                pass

    def _cluster_candidates(self, pcd):
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

    def _icp_for_candidate(self, candidate_points):
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

    def _match_roi_points(self, roi_points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(roi_points)
        pcd = pcd.voxel_down_sample(VOXEL_SIZE_SCENE)
        if len(pcd.points) < MIN_CLUSTER_POINTS:
            return None

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

        for cand in candidates:
            cand_points = np.asarray(cand.points, dtype=np.float64)
            if len(cand_points) < MIN_CLUSTER_POINTS:
                continue

            point_ratio = len(cand_points) / float(max(1, self.template_point_count))
            if point_ratio < POINT_COUNT_RATIO_MIN or point_ratio > POINT_COUNT_RATIO_MAX:
                continue

            cand_extent = self._pca_extent(cand_points)
            extent_ratio = cand_extent / self.template_pca_extent
            if np.any(extent_ratio < PCA_EXTENT_RATIO_MIN) or np.any(extent_ratio > PCA_EXTENT_RATIO_MAX):
                continue

            result = self._icp_for_candidate(cand_points)
            if result is None:
                continue

            tf_mat, fit, rmse = result
            if (fit > best_fit) or (abs(fit - best_fit) < 1e-6 and rmse < best_rmse):
                best_fit = fit
                best_rmse = rmse
                best_tf = tf_mat

        if best_tf is None or best_fit < MIN_SEARCH_FITNESS or best_rmse > MAX_SEARCH_RMSE:
            return None

        rot = best_tf[:3, :3]
        trans = best_tf[:3, 3]
        return trans, rot, best_fit, best_rmse, best_tf

    def _publish_detected_tf(self, translation, rotation, header):
        qx, qy, qz, qw = quat_from_rotmat(rotation)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = header.stamp
        tf_msg.header.frame_id = header.frame_id or "camera_depth_optical_frame"
        tf_msg.child_frame_id = DETECTED_FRAME
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)


# ===== Capture Node (integrated) =====
class D405CaptureAllInOneNode(Node):
    def __init__(self, matcher_node: TemplateMatchingRoiIcpNode) -> None:
        super().__init__("d405_capture_all_in_one_node")

        self.matcher_node = matcher_node
        self.pose_pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_marker_tf = None
        self.last_detected_tf = None
        self.last_tf_lock = threading.Lock()
        self.tf_republish_timer = self.create_timer(0.2, self._republish_last_tfs)
        self.capture_active = False
        self._shutdown = False

        self.get_logger().info("D405 All-in-One Capture Node gestartet")
        self.get_logger().info("Enter -> Capture | q + Enter -> Beenden")
        self.get_logger().info("All-in-One aktiv: eine Datei, Matcher intern, nur bei Enter aktiv")

        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()

    def _input_loop(self) -> None:
        while not self._shutdown and rclpy.ok():
            try:
                user = input("\n[Capture] Enter druecken fuer Aufnahme (q zum Beenden): ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if user == "q":
                self.get_logger().info("Beende auf Benutzerwunsch")
                rclpy.shutdown()
                return

            self._run_single_capture()

    def _run_single_capture(self) -> None:
        self.capture_active = True
        self.matcher_node.set_capture_enabled(True)

        self._publish_status("CAPTURE_START")
        self.get_logger().info("═══ CAPTURE START ═══")
        self.get_logger().info(f"Warte auf Pose (Timeout: {CAPTURE_TIMEOUT_SEC}s)...")

        success = self._wait_and_publish_pose(CAPTURE_TIMEOUT_SEC)

        self.matcher_node.set_capture_enabled(False)
        self.capture_active = False

        if success:
            self._publish_status("CAPTURE_OK")
            self.get_logger().info("═══ CAPTURE OK ═══")
        else:
            self._publish_status("CAPTURE_NOT_FOUND")
            self.get_logger().warn("✗ Keine Zange gefunden")
            print("Keine Zange gefunden")

    def _wait_and_publish_pose(self, timeout_sec: float) -> bool:
        t_end = time.time() + timeout_sec
        capture_start_stamp = self.get_clock().now().to_msg()
        log_once_marker = True
        log_once_detected = True

        while rclpy.ok() and time.time() < t_end:
            elapsed = time.time() - (t_end - timeout_sec)

            try:
                marker_tf = self.tf_buffer.lookup_transform(
                    "camera_color_optical_frame",
                    MARKER_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                if self._stamp_is_older_than(marker_tf.header.stamp, capture_start_stamp):
                    time.sleep(0.05)
                    continue
                if log_once_marker:
                    self.get_logger().info(f"✓ ArUco Marker erkannt: {MARKER_FRAME}")
                    log_once_marker = False
            except Exception:
                if log_once_marker:
                    self.get_logger().debug(f"[{elapsed:.1f}s] Warte auf {MARKER_FRAME}...")
                    log_once_marker = False
                time.sleep(0.05)
                continue

            try:
                detected_tf = self.tf_buffer.lookup_transform(
                    "camera_depth_optical_frame",
                    DETECTED_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                if self._stamp_is_older_than(detected_tf.header.stamp, capture_start_stamp):
                    time.sleep(0.05)
                    continue
                if log_once_detected:
                    self.get_logger().info(f"✓ Zange erkannt: {DETECTED_FRAME}")
                    log_once_detected = False
            except Exception:
                if log_once_detected:
                    self.get_logger().debug(f"[{elapsed:.1f}s] {MARKER_FRAME} da, warte auf {DETECTED_FRAME}...")
                    log_once_detected = False
                time.sleep(0.05)
                continue

            try:
                pose = self._compose_pose_in_marker_from_transforms(marker_tf, detected_tf)
                if pose is not None:
                    self._store_last_tfs(marker_tf, detected_tf)
                    self.pose_pub.publish(pose)
                    p = pose.pose.position
                    q = pose.pose.orientation
                    self.get_logger().info(
                        f"✓ Pose publiziert in {MARKER_FRAME}: "
                        f"x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}, "
                        f"qx={q.x:.4f}, qy={q.y:.4f}, qz={q.z:.4f}, qw={q.w:.4f}"
                    )
                    return True
            except Exception as exc:
                self.get_logger().error(f"Pose-Komposition fehlgeschlagen: {exc}")
                time.sleep(0.05)
                continue

            time.sleep(0.05)

        self.get_logger().warn(f"Timeout nach {timeout_sec}s")
        return False

    def _stamp_is_older_than(self, stamp_a, stamp_b) -> bool:
        if stamp_a.sec < stamp_b.sec:
            return True
        if stamp_a.sec > stamp_b.sec:
            return False
        return stamp_a.nanosec < stamp_b.nanosec

    def _store_last_tfs(self, marker_ts, detected_ts) -> None:
        marker_copy = TransformStamped()
        marker_copy.header.frame_id = marker_ts.header.frame_id
        marker_copy.child_frame_id = marker_ts.child_frame_id
        marker_copy.transform = marker_ts.transform

        detected_copy = TransformStamped()
        detected_copy.header.frame_id = detected_ts.header.frame_id
        detected_copy.child_frame_id = detected_ts.child_frame_id
        detected_copy.transform = detected_ts.transform

        with self.last_tf_lock:
            self.last_marker_tf = marker_copy
            self.last_detected_tf = detected_copy

        self.get_logger().info(
            f"✓ TF-Latch aktiv: {marker_copy.child_frame_id} und {detected_copy.child_frame_id} werden weiter publiziert"
        )

    def _republish_last_tfs(self) -> None:
        if self.capture_active:
            return

        with self.last_tf_lock:
            marker_tf = self.last_marker_tf
            detected_tf = self.last_detected_tf

        if marker_tf is None or detected_tf is None:
            return

        marker_out = TransformStamped()
        marker_out.header.stamp = marker_tf.header.stamp
        marker_out.header.frame_id = marker_tf.header.frame_id
        marker_out.child_frame_id = marker_tf.child_frame_id
        marker_out.transform = marker_tf.transform

        detected_out = TransformStamped()
        detected_out.header.stamp = detected_tf.header.stamp
        detected_out.header.frame_id = detected_tf.header.frame_id
        detected_out.child_frame_id = detected_tf.child_frame_id
        detected_out.transform = detected_tf.transform

        self.tf_broadcaster.sendTransform(marker_out)
        self.tf_broadcaster.sendTransform(detected_out)

    def _compose_pose_in_marker_from_transforms(self, marker_ts, detected_ts) -> Optional[PoseStamped]:
        marker_parent = marker_ts.header.frame_id
        detected_parent = detected_ts.header.frame_id

        if marker_parent != detected_parent:
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    detected_pose = PoseStamped()
                    detected_pose.header.frame_id = detected_parent
                    detected_pose.header.stamp = detected_ts.header.stamp
                    detected_pose.pose.position.x = detected_ts.transform.translation.x
                    detected_pose.pose.position.y = detected_ts.transform.translation.y
                    detected_pose.pose.position.z = detected_ts.transform.translation.z
                    detected_pose.pose.orientation.x = detected_ts.transform.rotation.x
                    detected_pose.pose.orientation.y = detected_ts.transform.rotation.y
                    detected_pose.pose.orientation.z = detected_ts.transform.rotation.z
                    detected_pose.pose.orientation.w = detected_ts.transform.rotation.w

                    detected_transformed = self.tf_buffer.transform(
                        detected_pose, marker_parent, timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    da = detected_transformed.pose.position
                    dq = detected_transformed.pose.orientation
                    break
                except Exception:
                    if attempt < max_retries - 1:
                        time.sleep(0.1)
                    else:
                        return None
        else:
            da = detected_ts.transform.translation
            dq = detected_ts.transform.rotation

        ma = marker_ts.transform.translation
        mq = marker_ts.transform.rotation

        p_m = np.array([ma.x, ma.y, ma.z], dtype=np.float64)
        p_d = np.array([da.x, da.y, da.z], dtype=np.float64)

        r_m = quat_to_rotmat(mq.x, mq.y, mq.z, mq.w)
        r_d = quat_to_rotmat(dq.x, dq.y, dq.z, dq.w)

        r_rel = r_m.T @ r_d
        p_rel = r_m.T @ (p_d - p_m)

        qx, qy, qz, qw = rotmat_to_quat(r_rel)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = MARKER_FRAME
        msg.pose.position.x = float(p_rel[0])
        msg.pose.position.y = float(p_rel[1])
        msg.pose.position.z = float(p_rel[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def destroy_node(self):
        self._shutdown = True
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    matcher_node = TemplateMatchingRoiIcpNode()
    capture_node = D405CaptureAllInOneNode(matcher_node)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(matcher_node)
    executor.add_node(capture_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        capture_node.destroy_node()
        matcher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
