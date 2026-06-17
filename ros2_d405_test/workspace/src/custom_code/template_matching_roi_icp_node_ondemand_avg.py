#!/usr/bin/env python3
"""
D405 ArUco -> Enter -> Tool (Mittelwert ueber N Scans)

Ablauf:
1) Beim Start wird ArUco erkannt und als TF (aruco_0) publiziert.
2) Nach erster ArUco-Erkennung wartet der Node auf Enter.
3) Nach Enter werden SCAN_AVERAGE_N gueltige ICP-Ergebnisse gesammelt
   und die gemittelte Pose publiziert.
4) Danach wird die Zangen-Erkennung gestoppt (kein Dauerbetrieb).
"""

import os
os.environ.setdefault("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}]: {message}")
import threading
import time
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


# Topics / Frames
POINTCLOUD_TOPIC = "/camera/camera/depth/color/points"
IMAGE_TOPIC = "/camera/camera/color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
TARGET_TOPIC = "/tool_target_pose"
STATUS_TOPIC = "/tool_detection_status"

MARKER_FRAME = "aruco_0"
DETECTED_FRAME = "detected_cropv1_center"

# ArUco
MARKER_ID = 0
MARKER_SIZE_M = 0.05
ARUCO_FILTER_ALPHA = 0.12
ARUCO_MAX_JUMP_M = 0.02
ARUCO_DEADBAND_M = 0.0015
DETECTED_LATCH_PUBLISH_RATE_HZ = 10.0

# One-shot tool scan timeout
TOOL_TIMEOUT_SEC = 12.0
MIN_CLOUD_STAMP_AFTER_SCAN_SEC = 0.10

# Template / ROI / ICP
TEMPLATE_DIR = "/workspace/src/custom_packages/custom_code/templates"
TEMPLATE_SCAN_LIST = [
    "breitv1_clean_direction",
    "kurzv3_clean_direction",
    "langv1_clean_direction",
]
CROP_BOUNDS_MARKER = (-0.03, 0.38, -0.22, 0.03, 0.01, 0.03)
ROI_Z_ADJUST_START_M = 0.000
ROI_Z_ADJUST_STEP_M = 0.002
ROI_Z_MAX_POINTS = 8000

SCAN_AVERAGE_N = 3      # Anzahl gueltige Messungen die gemittelt werden
MATCH_INTERVAL = 0.10   # Sekunden zwischen Messungen (3x = ~0.3s gesamt)
MAX_RAW_POINTS = 60000
VOXEL_SIZE_SCENE = 0.0025
VOXEL_SIZE_TEMPLATE = 0.0025


CLUSTER_EPS = 0.012
CLUSTER_MIN_POINTS = 30
MIN_CLUSTER_POINTS = 90
MAX_CLUSTERS_TO_TEST = 4

ICP_THRESHOLD = 0.0075
ICP_MAX_ITER = 60
MIN_SEARCH_FITNESS = 0.85
MAX_SEARCH_RMSE = 0.10

POINT_COUNT_RATIO_MIN = 0.40
POINT_COUNT_RATIO_MAX = 2.50
PCA_EXTENT_RATIO_MIN = 0.65
PCA_EXTENT_RATIO_MAX = 1.55

DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
}


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


class D405ArucoThenToolOnceNode(Node):
    def __init__(self) -> None:
        super().__init__("d405_aruco_then_tool_avg_node")

        self.pose_pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.k = None
        self.dist = None
        self.camera_frame = "camera_color_optical_frame"

        self.aruco_found = False
        self.wait_for_enter = False
        self.scan_active = False
        self.scan_deadline = 0.0
        self.scan_start_ros_time = 0.0
        self._cached_z_min = None
        self.last_cloud_process_time = 0.0
        self.latest_cloud_msg = None
        self.cloud_lock = threading.Lock()
        self._scan_results: list = []
        self._locked_template_id: str = ""

        self.marker_position = None
        self.marker_rotation = None
        self.marker_filtered_tvec = None
        self.marker_filtered_rvec = None
        self.marker_lock = threading.Lock()
        self.last_detected_tf = None
        self.detected_lock = threading.Lock()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP["DICT_4X4_50"])
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        self.loaded_templates: Dict[str, Tuple[np.ndarray, int, np.ndarray]] = {}
        if not self._load_templates():
            self.get_logger().error("Keine Templates geladen. Node bleibt ohne Zangen-Erkennung.")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, qos_sensor)
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, qos_sensor)
        self.create_subscription(PointCloud2, POINTCLOUD_TOPIC, self._on_cloud, qos_sensor)
        self.create_subscription(Bool, "/gui/scan_trigger", self._on_scan_trigger, 10)

        self._shutdown = False
        self.cloud_worker_thread = threading.Thread(target=self._cloud_worker, daemon=True)
        self.cloud_worker_thread.start()
        self.detected_latch_thread = threading.Thread(target=self._detected_latch_worker, daemon=True)
        self.detected_latch_thread.start()

        self.get_logger().info(f"Node gestartet (AVG-Modus: {SCAN_AVERAGE_N} Messungen): zuerst ArUco, dann Enter fuer Zangen-Erkennung")

    def _load_templates(self) -> bool:
        self.loaded_templates = {}
        for template_id in TEMPLATE_SCAN_LIST:
            template_path = os.path.join(TEMPLATE_DIR, f"{template_id}.pcd")
            try:
                pcd = o3d.io.read_point_cloud(template_path)
            except Exception as exc:
                self.get_logger().warn(f"Template konnte nicht geladen werden: {template_id}: {exc}")
                continue

            if len(pcd.points) < 50:
                self.get_logger().warn(f"Template zu klein/ungueltig: {template_id}")
                continue

            pcd = pcd.voxel_down_sample(VOXEL_SIZE_TEMPLATE)
            pts = np.asarray(pcd.points, dtype=np.float64)
            centroid = pts.mean(axis=0)
            template_points = pts - centroid
            template_point_count = len(template_points)
            template_pca_extent = self._pca_extent(template_points)

            self.loaded_templates[template_id] = (
                template_points,
                template_point_count,
                template_pca_extent,
            )
            self.get_logger().info(f"Template geladen: {template_id} ({template_point_count} Punkte)")

        self.get_logger().info(f"Templates aktiv: {list(self.loaded_templates.keys())}")
        return len(self.loaded_templates) > 0

    def _pca_extent(self, points: np.ndarray) -> np.ndarray:
        if len(points) < 10:
            return np.array([1e-6, 1e-6, 1e-6], dtype=np.float64)
        r = pca_rotation(points)
        centered = points - points.mean(axis=0)
        proj = centered @ r
        ext = np.ptp(proj, axis=0)
        return np.maximum(ext, 1e-6)

    def _on_camera_info(self, msg: CameraInfo) -> None:
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
        expected_rgb = msg.height * msg.width * 3

        if msg.encoding in ("bgr8", "rgb8"):
            if data.size < expected_rgb:
                return None
            img = data[:expected_rgb].reshape((msg.height, msg.width, 3))
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img

        if msg.encoding == "mono8":
            expected_mono = msg.height * msg.width
            if data.size < expected_mono:
                return None
            mono = data[:expected_mono].reshape((msg.height, msg.width))
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

        return None

    def _on_image(self, msg: Image) -> None:
        if self.k is None:
            return

        frame = self._image_to_bgr8(msg)
        if frame is None:
            return

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_M, self.k, self.dist)

        for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
            if int(marker_id) != MARKER_ID:
                continue

            raw_rvec = np.array(rvec[0], dtype=np.float64).reshape(3)
            raw_tvec = np.array(tvec[0], dtype=np.float64).reshape(3)

            # ArUco Pose glätten und Ausreißer dämpfen.
            if self.marker_filtered_tvec is None:
                filt_tvec = raw_tvec
                filt_rvec = raw_rvec
            else:
                jump = np.linalg.norm(raw_tvec - self.marker_filtered_tvec)
                if jump > ARUCO_MAX_JUMP_M:
                    filt_tvec = self.marker_filtered_tvec
                    filt_rvec = self.marker_filtered_rvec
                else:
                    if jump < ARUCO_DEADBAND_M:
                        filt_tvec = self.marker_filtered_tvec
                        filt_rvec = self.marker_filtered_rvec
                    else:
                        if np.dot(raw_rvec, self.marker_filtered_rvec) < 0.0:
                            raw_rvec = -raw_rvec
                        filt_tvec = (
                            ARUCO_FILTER_ALPHA * raw_tvec
                            + (1.0 - ARUCO_FILTER_ALPHA) * self.marker_filtered_tvec
                        )
                        filt_rvec = (
                            ARUCO_FILTER_ALPHA * raw_rvec
                            + (1.0 - ARUCO_FILTER_ALPHA) * self.marker_filtered_rvec
                        )

            self.marker_filtered_tvec = filt_tvec
            self.marker_filtered_rvec = filt_rvec

            r_mat, _ = cv2.Rodrigues(filt_rvec)
            qx, qy, qz, qw = rotmat_to_quat(r_mat)
            t = np.array(filt_tvec, dtype=np.float64).reshape(3)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.camera_frame
            tf_msg.child_frame_id = MARKER_FRAME
            tf_msg.transform.translation.x = float(t[0])
            tf_msg.transform.translation.y = float(t[1])
            tf_msg.transform.translation.z = float(t[2])
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

            with self.marker_lock:
                self.marker_position = np.array(t, dtype=np.float64)
                self.marker_rotation = np.array(r_mat, dtype=np.float64)

            if not self.aruco_found:
                self.aruco_found = True
                self.wait_for_enter = True
                self.get_logger().info(f"✓ ArUco erkannt und publiziert: {MARKER_FRAME}")
                self.get_logger().info("Jetzt Enter druecken fuer Zangen-Erkennung")
            return

    def _on_scan_trigger(self, msg: Bool) -> None:
        if not msg.data:
            return
        if not self.aruco_found:
            self.get_logger().warn("[GUI-Trigger] ArUco noch nicht erkannt — Scan ignoriert")
            return
        self.get_logger().info("[GUI-Trigger] Scan-Trigger empfangen, starte Zangen-Erkennung")
        self._start_tool_scan_once()

    def _start_tool_scan_once(self) -> None:
        if self.scan_active:
            self.get_logger().info("Scan laeuft bereits")
            return

        # Nur frische Clouds nach dem Enter-Trigger fuer den aktuellen Scan zulassen.
        with self.cloud_lock:
            self.latest_cloud_msg = None

        with self.detected_lock:
            self.last_detected_tf = None

        self._scan_results = []
        self._locked_template_id = ""
        self.scan_active = True
        self.scan_deadline = time.time() + TOOL_TIMEOUT_SEC
        self.scan_start_ros_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_cloud_process_time = 0.0
        self._cached_z_min = None
        self._publish_status("TOOL_SCAN_START")
        self.get_logger().info("═══ TOOL SCAN START ═══")

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not self.scan_active:
            return
        with self.cloud_lock:
            self.latest_cloud_msg = msg

    def _cloud_worker(self) -> None:
        while not self._shutdown and rclpy.ok():
            if not self.scan_active:
                time.sleep(0.02)
                continue

            if time.time() > self.scan_deadline:
                self.scan_active = False
                self._publish_status("TOOL_SCAN_NOT_FOUND")
                self.get_logger().warn(f"Timeout nach {TOOL_TIMEOUT_SEC:.1f}s")
                self.get_logger().warn("✗ Keine Zange gefunden")
                continue

            now = time.time()
            if now - self.last_cloud_process_time < MATCH_INTERVAL:
                time.sleep(0.005)
                continue

            with self.cloud_lock:
                cloud_msg = self.latest_cloud_msg
                self.latest_cloud_msg = None

            if cloud_msg is None:
                time.sleep(0.005)
                continue

            cloud_stamp_sec = stamp_to_seconds(cloud_msg.header.stamp)
            if cloud_stamp_sec > 0.0:
                min_allowed_stamp = self.scan_start_ros_time + MIN_CLOUD_STAMP_AFTER_SCAN_SEC
                if cloud_stamp_sec < min_allowed_stamp:
                    time.sleep(0.002)
                    continue

            self.last_cloud_process_time = now

            try:
                self._process_cloud_msg(cloud_msg)
            except Exception as exc:
                self.get_logger().debug(f"Tool-Scan Debug: {exc}")

    def _process_cloud_msg(self, msg: PointCloud2) -> None:
        pts = [[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
        if len(pts) < MIN_CLUSTER_POINTS:
            return

        points_np = np.asarray(pts, dtype=np.float64)
        if len(points_np) > MAX_RAW_POINTS:
            step = max(1, len(points_np) // MAX_RAW_POINTS)
            points_np = points_np[::step]

        crop_result = self._crop_roi(points_np, msg.header.frame_id)
        if crop_result is None:
            return
        roi_points, p_marker, r_marker = crop_result

        best_result = None
        best_template_id = ""
        best_fit = 0.0
        best_rmse = float("inf")

        # Ab Messung 2: nur noch das bereits gewählte Template verwenden.
        if self._locked_template_id:
            templates_to_check = {
                self._locked_template_id: self.loaded_templates[self._locked_template_id]
            }
        else:
            templates_to_check = self.loaded_templates

        for template_id, (template_points, template_point_count, template_pca_extent) in templates_to_check.items():
            result, fit, rmse = self._match_roi_points_for_template(
                roi_points,
                template_points,
                template_point_count,
                template_pca_extent,
            )

            rmse_str = f"{rmse:.4f}" if np.isfinite(rmse) else "n/a"
            self.get_logger().info(f"  {template_id}: fitness={fit:.4f}, rmse={rmse_str}")

            if result is None:
                continue

            if (fit > best_fit) or (abs(fit - best_fit) < 1e-6 and rmse < best_rmse):
                best_result = result
                best_fit = fit
                best_rmse = rmse
                best_template_id = template_id

        if best_result is None:
            return

        translation, rotation, _, _, _ = best_result

        # Direkte Berechnung marker->detected im Pointcloud-Frame.
        r_rel = r_marker.T @ rotation
        p_rel = r_marker.T @ (translation - p_marker)
        qx, qy, qz, qw = rotmat_to_quat(r_rel)

        # Nach erster Messung: Template festlegen für die Folgemessungen.
        if not self._locked_template_id:
            self._locked_template_id = best_template_id
            self.get_logger().info(f"[AVG] Template festgelegt: {best_template_id}")

        self._scan_results.append({
            "p_rel": p_rel.copy(),
            "quat": np.array([qx, qy, qz, qw], dtype=np.float64),
            "r_rel": r_rel.copy(),
            "template_id": best_template_id,
            "fitness": best_fit,
            "rmse": best_rmse,
        })

        n = len(self._scan_results)
        self.get_logger().info(
            f"[AVG {n}/{SCAN_AVERAGE_N}] {best_template_id} fitness={best_fit:.4f}, rmse={best_rmse:.4f} | "
            f"x={p_rel[0]:.4f}, y={p_rel[1]:.4f}, z={p_rel[2]:.4f}"
        )

        if n < SCAN_AVERAGE_N:
            return

        # Genug Messungen gesammelt — Mittelwert berechnen.
        avg_p = np.mean([r["p_rel"] for r in self._scan_results], axis=0)

        # Quaternion-Mittelwert: Vorzeichen an erstes Ergebnis angleichen, dann normieren.
        quats = np.array([r["quat"] for r in self._scan_results], dtype=np.float64)
        ref = quats[0]
        for i in range(1, len(quats)):
            if np.dot(quats[i], ref) < 0.0:
                quats[i] = -quats[i]
        avg_q = quats.mean(axis=0)
        avg_q /= np.linalg.norm(avg_q)
        aqx, aqy, aqz, aqw = float(avg_q[0]), float(avg_q[1]), float(avg_q[2]), float(avg_q[3])

        # Rotation aus gemitteltem Quaternion fuer TF.
        avg_r = quat_to_rotmat(aqx, aqy, aqz, aqw)
        self._publish_detected_tf(avg_p, avg_r)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        template_ids = [r["template_id"] for r in self._scan_results]
        dominant_template = max(set(template_ids), key=template_ids.count)
        pose.header.frame_id = f"{MARKER_FRAME}|{dominant_template}"
        pose.pose.position.x = float(avg_p[0])
        pose.pose.position.y = float(avg_p[1])
        pose.pose.position.z = float(avg_p[2])
        pose.pose.orientation.x = aqx
        pose.pose.orientation.y = aqy
        pose.pose.orientation.z = aqz
        pose.pose.orientation.w = aqw

        self.pose_pub.publish(pose)
        p = pose.pose.position
        avg_fitness = np.mean([r["fitness"] for r in self._scan_results])
        avg_rmse = np.mean([r["rmse"] for r in self._scan_results])
        self.get_logger().info(
            f"✓ Zange erkannt (Mittelwert {SCAN_AVERAGE_N} Messungen) in {MARKER_FRAME}: "
            f"x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}, "
            f"qx={aqx:.4f}, qy={aqy:.4f}, qz={aqz:.4f}, qw={aqw:.4f}"
        )
        self.get_logger().info(
            f"Template: {dominant_template} (avg fitness={avg_fitness:.4f}, avg rmse={avg_rmse:.4f})"
        )

        self.scan_active = False
        self._publish_status("TOOL_SCAN_OK")
        self.get_logger().info("═══ TOOL SCAN OK ═══")
        self.get_logger().info("Bereit fuer naechsten Enter-Scan")

    def _crop_roi(self, points: np.ndarray, cloud_frame: str):
        try:
            marker_tf = self.tf_buffer.lookup_transform(
                cloud_frame,
                MARKER_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            marker_pos = np.array([
                marker_tf.transform.translation.x,
                marker_tf.transform.translation.y,
                marker_tf.transform.translation.z,
            ], dtype=np.float64)
            mq = marker_tf.transform.rotation
            marker_rot = quat_to_rotmat(mq.x, mq.y, mq.z, mq.w)
        except Exception:
            with self.marker_lock:
                marker_pos = None if self.marker_position is None else self.marker_position.copy()
                marker_rot = None if self.marker_rotation is None else self.marker_rotation.copy()

            if marker_pos is None or marker_rot is None:
                return None

        x_min, x_max, y_min, y_max, _, z_max = CROP_BOUNDS_MARKER
        points_rel = points - marker_pos
        points_marker = points_rel @ marker_rot.T

        if self._cached_z_min is None:
            z_min = ROI_Z_ADJUST_START_M
            roi = np.empty((0, 3), dtype=np.float64)
            while z_min < z_max:
                mask = (
                    (points_marker[:, 0] >= x_min)
                    & (points_marker[:, 0] <= x_max)
                    & (points_marker[:, 1] >= y_min)
                    & (points_marker[:, 1] <= y_max)
                    & (points_marker[:, 2] >= z_min)
                    & (points_marker[:, 2] <= z_max)
                )
                roi = points[mask]
                if len(roi) <= ROI_Z_MAX_POINTS:
                    break
                self.get_logger().info(
                    f"[ROI_Z_ADJ] {len(roi)} Punkte > {ROI_Z_MAX_POINTS}, z_min {z_min:.3f} -> {z_min + ROI_Z_ADJUST_STEP_M:.3f}"
                )
                z_min += ROI_Z_ADJUST_STEP_M
            self._cached_z_min = z_min
            self.get_logger().info(f"[ROI_Z_ADJ] Verwende z_min={z_min:.3f}, ROI-Punkte={len(roi)}")
        else:
            z_min = self._cached_z_min
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

    def _icp_for_candidate(self, candidate_points: np.ndarray, template_points: np.ndarray):
        if template_points is None or len(template_points) == 0:
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
            source.points = o3d.utility.Vector3dVector(template_points)
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

        # Reverse fitness: how well does the template cover the candidate?
        # Catches the case where a short template fits into a larger object.
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(template_points)
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(candidate_points)
        t_inv = np.linalg.inv(best_tf)
        rev = o3d.pipelines.registration.evaluate_registration(
            target, source, ICP_THRESHOLD, t_inv
        )
        # Combined: geometric mean of forward and reverse fitness
        combined_fit = float(np.sqrt(best_fit * max(rev.fitness, 1e-6)))

        return best_tf, combined_fit, best_rmse

    def _match_roi_points_for_template(
        self,
        roi_points: np.ndarray,
        template_points: np.ndarray,
        template_point_count: int,
        template_pca_extent: np.ndarray,
    ):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(roi_points)
        pcd = pcd.voxel_down_sample(VOXEL_SIZE_SCENE)
        if len(pcd.points) < MIN_CLUSTER_POINTS:
            self.get_logger().info(
                f"ICP Diagnose: zu wenige ROI-Punkte nach Voxel ({len(pcd.points)} < {MIN_CLUSTER_POINTS})"
            )
            return None, 0.0, float("inf")

        candidates = self._cluster_candidates(pcd)
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

            point_ratio = len(cand_points) / float(max(1, template_point_count))
            if point_ratio < POINT_COUNT_RATIO_MIN or point_ratio > POINT_COUNT_RATIO_MAX:
                reject_point_ratio += 1
                continue

            cand_extent = self._pca_extent(cand_points)
            extent_ratio = cand_extent / template_pca_extent
            if np.any(extent_ratio < PCA_EXTENT_RATIO_MIN) or np.any(extent_ratio > PCA_EXTENT_RATIO_MAX):
                reject_extent += 1
                continue

            icp_attempts += 1
            result = self._icp_for_candidate(cand_points, template_points)
            if result is None:
                continue

            tf_mat, fit, rmse = result
            if (fit > best_fit) or (abs(fit - best_fit) < 1e-6 and rmse < best_rmse):
                best_fit = fit
                best_rmse = rmse
                best_tf = tf_mat

        if best_tf is None and len(candidates) > 0:
            # Fallback: Wenn alle Kandidaten an Vorfiltern scheitern, trotzdem ICP auf Clustern versuchen.
            for cand in candidates:
                cand_points = np.asarray(cand.points, dtype=np.float64)
                if len(cand_points) < MIN_CLUSTER_POINTS:
                    continue
                icp_attempts += 1
                result = self._icp_for_candidate(cand_points, template_points)
                if result is None:
                    continue
                tf_mat, fit, rmse = result
                if (fit > best_fit) or (abs(fit - best_fit) < 1e-6 and rmse < best_rmse):
                    best_fit = fit
                    best_rmse = rmse
                    best_tf = tf_mat

            self.get_logger().warn("ICP: Fallback ohne Shape-Filter")

        self.get_logger().info(f"ROI: raw={len(roi_points)}, ds={len(pcd.points)}")
        rejects = []
        if reject_small:       rejects.append(f"small={reject_small}")
        if reject_point_ratio: rejects.append(f"ratio={reject_point_ratio}")
        if reject_extent:      rejects.append(f"extent={reject_extent}")
        if rejects:
            self.get_logger().info(f"ICP verworfen: {', '.join(rejects)}")

        if best_tf is None:
            return None, best_fit, best_rmse

        if best_fit < MIN_SEARCH_FITNESS or best_rmse > MAX_SEARCH_RMSE:
            return None, best_fit, best_rmse

        rot = best_tf[:3, :3]
        trans = best_tf[:3, 3]
        return (trans, rot, best_fit, best_rmse, best_tf), best_fit, best_rmse

    def _publish_detected_tf(self, translation_marker: np.ndarray, rotation_marker: np.ndarray):
        """Publiziert Zangen-TF direkt unter aruco_0 und merkt ihn fuer Latch-Republish."""
        qx, qy, qz, qw = rotmat_to_quat(rotation_marker)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = MARKER_FRAME
        tf_msg.child_frame_id = DETECTED_FRAME
        tf_msg.transform.translation.x = float(translation_marker[0])
        tf_msg.transform.translation.y = float(translation_marker[1])
        tf_msg.transform.translation.z = float(translation_marker[2])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        with self.detected_lock:
            self.last_detected_tf = tf_msg

    def _republish_last_detected_tf(self) -> None:
        with self.detected_lock:
            tf_last = self.last_detected_tf

        if tf_last is None:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = tf_last.header.frame_id
        tf_msg.child_frame_id = tf_last.child_frame_id
        tf_msg.transform = tf_last.transform
        self.tf_broadcaster.sendTransform(tf_msg)

    def _detected_latch_worker(self) -> None:
        period = 1.0 / max(1e-6, DETECTED_LATCH_PUBLISH_RATE_HZ)
        while not self._shutdown and rclpy.ok():
            time.sleep(period)
            try:
                self._republish_last_detected_tf()
            except Exception as exc:
                self.get_logger().debug(f"Detected-Latch Fehler: {exc}")

    def _compose_pose_in_marker(self, marker_ts, detected_ts) -> Optional[PoseStamped]:
        marker_parent = marker_ts.header.frame_id
        detected_parent = detected_ts.header.frame_id

        if marker_parent != detected_parent:
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

            try:
                detected_transformed = self.tf_buffer.transform(
                    detected_pose, marker_parent, timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except Exception:
                return None

            da = detected_transformed.pose.position
            dq = detected_transformed.pose.orientation
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
    node = D405ArucoThenToolOnceNode()
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
