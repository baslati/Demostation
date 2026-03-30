#!/usr/bin/env python3
"""
Single-Template ICP Matching fuer eine Zange.

Ziel: robuste Pose-Schaetzung mit moderater CPU-Last.
Ansatz:
    1) Punktwolke ausduennen
    2) Tisch entfernen
    3) Groessten Cluster als Objekt verwenden
    4) Initiale Ausrichtung ueber Centroid + PCA
    5) Point-to-Point ICP gegen ein einziges Template
    6) TF publizieren
"""
import os
import time
import threading

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPEN3D_CPU_THREAD_COUNT", "1")

import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from tf2_ros import TransformBroadcaster


POINTCLOUD_TOPIC = "/camera/camera/depth/color/points"
TEMPLATE_PATH = "/workspace/src/custom_packages/custom_code/templates/cropv1_clean.pcd"
IMAGE_TOPIC = "/camera/camera/color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"

MARKER_SIZE = 0.05
MARKER_ID_FOR_FRAME = 0
FRAME_PREFIX = "aruco_"

# ROI fuer Vor-Crop im Marker-Frame: x_min x_max y_min y_max z_min z_max
CROP_BOUNDS_MARKER = (-0.03, 0.38, -0.22, 0.03, 0.0, 0.03)

DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

MATCH_INTERVAL = 0.8
MAX_RAW_POINTS = 25000
VOXEL_SIZE = 0.003

RANSAC_DISTANCE_THRESHOLD = 0.010
RANSAC_N = 3
RANSAC_ITERATIONS = 180

CLUSTER_EPS = 0.015
CLUSTER_MIN_POINTS = 35
MIN_CLUSTER_POINTS = 80

ICP_MAX_CORR_DIST = 0.010
ICP_MAX_ITER = 20
MIN_FITNESS = 0.20

POSE_SMOOTH_ALPHA = 0.35


def quat_from_rotmat(r_mat):
    """Konvertiert 3x3 Rotationsmatrix in Quaternion (x, y, z, w)."""
    trace = float(r_mat[0, 0] + r_mat[1, 1] + r_mat[2, 2])

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (r_mat[2, 1] - r_mat[1, 2]) * s
        qy = (r_mat[0, 2] - r_mat[2, 0]) * s
        qz = (r_mat[1, 0] - r_mat[0, 1]) * s
    elif r_mat[0, 0] > r_mat[1, 1] and r_mat[0, 0] > r_mat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r_mat[0, 0] - r_mat[1, 1] - r_mat[2, 2])
        qw = (r_mat[2, 1] - r_mat[1, 2]) / s
        qx = 0.25 * s
        qy = (r_mat[0, 1] + r_mat[1, 0]) / s
        qz = (r_mat[0, 2] + r_mat[2, 0]) / s
    elif r_mat[1, 1] > r_mat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r_mat[1, 1] - r_mat[0, 0] - r_mat[2, 2])
        qw = (r_mat[0, 2] - r_mat[2, 0]) / s
        qx = (r_mat[0, 1] + r_mat[1, 0]) / s
        qy = 0.25 * s
        qz = (r_mat[1, 2] + r_mat[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r_mat[2, 2] - r_mat[0, 0] - r_mat[1, 1])
        qw = (r_mat[1, 0] - r_mat[0, 1]) / s
        qx = (r_mat[0, 2] + r_mat[2, 0]) / s
        qy = (r_mat[1, 2] + r_mat[2, 1]) / s
        qz = 0.25 * s

    return float(qx), float(qy), float(qz), float(qw)


def rotation_from_pca(points):
    """Bestimmt eine rechtshaendige Rotationsmatrix aus PCA-Hauptachsen."""
    centered = points - points.mean(axis=0)
    cov = centered.T @ centered / max(1, len(centered) - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    r_mat = eigvecs[:, order]

    # Vorzeichen konsistent halten (reduziert Achsenspruenge zwischen Frames)
    if r_mat[0, 0] < 0.0:
        r_mat[:, 0] *= -1.0
    if r_mat[1, 1] < 0.0:
        r_mat[:, 1] *= -1.0

    # Rechtshaendig erzwingen
    r_mat[:, 2] = np.cross(r_mat[:, 0], r_mat[:, 1])
    r_mat[:, 2] /= np.linalg.norm(r_mat[:, 2]) + 1e-12
    r_mat[:, 1] = np.cross(r_mat[:, 2], r_mat[:, 0])
    r_mat[:, 1] /= np.linalg.norm(r_mat[:, 1]) + 1e-12

    return r_mat


def rigid_transform_svd(source, target):
    """Berechnet starre Transformation (R, t) fuer korrespondierende Punkte."""
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)

    source_centered = source - source_centroid
    target_centered = target - target_centroid

    h_mat = source_centered.T @ target_centered
    u_mat, _, v_t = np.linalg.svd(h_mat)
    r_mat = v_t.T @ u_mat.T

    if np.linalg.det(r_mat) < 0:
        v_t[2, :] *= -1
        r_mat = v_t.T @ u_mat.T

    t_vec = target_centroid - (r_mat @ source_centroid)
    return r_mat, t_vec


def nearest_neighbors_numpy(source_points, target_points):
    """Naechster Nachbar je Source-Punkt mit NumPy."""
    src_sq = np.sum(source_points * source_points, axis=1, keepdims=True)
    tgt_sq = np.sum(target_points * target_points, axis=1)
    d2 = src_sq + tgt_sq[None, :] - 2.0 * (source_points @ target_points.T)
    np.maximum(d2, 0.0, out=d2)
    indices = np.argmin(d2, axis=1)
    dists = np.sqrt(d2[np.arange(len(source_points)), indices])
    return dists, indices


def icp_point_to_point(source_points, target_points, max_corr_dist, max_iterations):
    """Einfache, schnelle ICP-Variante."""
    if len(source_points) < 30 or len(target_points) < 30:
        return np.eye(4), 0.0, float("inf")

    src = source_points.copy()
    t_total = np.eye(4)

    # grobe Initialisierung ueber Zentroiden
    shift = target_points.mean(axis=0) - src.mean(axis=0)
    src += shift
    t_total[:3, 3] = shift

    prev_rmse = float("inf")
    fitness = 0.0
    rmse = float("inf")

    for _ in range(max_iterations):
        dists, indices = nearest_neighbors_numpy(src, target_points)
        valid = dists < max_corr_dist
        valid_count = int(valid.sum())
        if valid_count < 30:
            break

        src_corr = src[valid]
        tgt_corr = target_points[indices[valid]]
        r_mat, t_vec = rigid_transform_svd(src_corr, tgt_corr)

        src = (r_mat @ src.T).T + t_vec

        step = np.eye(4)
        step[:3, :3] = r_mat
        step[:3, 3] = t_vec
        t_total = step @ t_total

        rmse = float(np.sqrt(np.mean(dists[valid] ** 2)))
        fitness = valid_count / float(len(source_points))
        if abs(prev_rmse - rmse) < 1e-5:
            break
        prev_rmse = rmse

    return t_total, fitness, rmse


class LightweightPliersPoseNode(Node):
    def __init__(self):
        super().__init__("template_matching_light_node")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_match_time = 0.0
        self.last_translation = None
        self.last_rotation = None
        self._marker_lock = threading.Lock()
        self.marker_position = None
        self.marker_rotation = None
        self._frame_warned = False

        # ArUco-Erkennung
        self.bridge = CvBridge()
        self.k = None
        self.dist = None
        self.camera_frame = "camera_color_optical_frame"
        self.target_marker_id = MARKER_ID_FOR_FRAME
        dict_name = "DICT_4X4_50"
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[dict_name])
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        loaded = self._load_single_template()
        if not loaded:
            self.get_logger().error("Template konnte nicht geladen werden. Node beendet sich.")
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            POINTCLOUD_TOPIC,
            self.listener_callback,
            qos,
        )
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, 10)
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, 10)

        self.get_logger().info("Leichtgewichtiges Template Matching gestartet")
        self.get_logger().info(f"Template: {TEMPLATE_PATH}")
        self.get_logger().info(f"Topic: {POINTCLOUD_TOPIC}")
        self.get_logger().info(
            f"ArUco aktiv: Marker {self.target_marker_id} auf {IMAGE_TOPIC} -> Frame {FRAME_PREFIX}{self.target_marker_id}"
        )
        self.get_logger().info(
            "ROI Marker-Frame: "
            f"x[{CROP_BOUNDS_MARKER[0]:.3f},{CROP_BOUNDS_MARKER[1]:.3f}] "
            f"y[{CROP_BOUNDS_MARKER[2]:.3f},{CROP_BOUNDS_MARKER[3]:.3f}] "
            f"z[{CROP_BOUNDS_MARKER[4]:.3f},{CROP_BOUNDS_MARKER[5]:.3f}] m"
        )

    def _load_single_template(self):
        if not os.path.exists(TEMPLATE_PATH):
            self.get_logger().error(f"Template fehlt: {TEMPLATE_PATH}")
            return False

        pcd = o3d.io.read_point_cloud(TEMPLATE_PATH)
        if len(pcd.points) < 20:
            self.get_logger().error("Template ist leer oder zu klein.")
            return False

        pcd_down = pcd.voxel_down_sample(VOXEL_SIZE)
        if len(pcd_down.points) < 30:
            pcd_down = pcd

        points = np.asarray(pcd_down.points, dtype=np.float64)
        template_centroid = points.mean(axis=0)
        centered_points = points - template_centroid

        self.template_points = centered_points
        self.template_extent = np.maximum(np.ptp(centered_points, axis=0), 1e-6)

        self.get_logger().info(
            f"Template geladen: {len(points)} Punkte | Extent x={self.template_extent[0]:.3f}, "
            f"y={self.template_extent[1]:.3f}, z={self.template_extent[2]:.3f} m"
        )
        self.get_logger().info(
            f"Template-Zentrierung (runtime): [{template_centroid[0]:.4f}, "
            f"{template_centroid[1]:.4f}, {template_centroid[2]:.4f}] m"
        )
        return True

    def _on_camera_info(self, msg: CameraInfo):
        if self.k is not None:
            return
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id or self.camera_frame
        self.get_logger().info(f"CameraInfo erhalten, ArUco-Frame: {self.camera_frame}")

    def _on_image(self, msg: Image):
        if self.k is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE, self.k, self.dist
        )

        for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
            if int(marker_id) != self.target_marker_id:
                continue

            r_mat, _ = cv2.Rodrigues(rvec[0])
            qx, qy, qz, qw = quat_from_rotmat(r_mat)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.camera_frame
            tf_msg.child_frame_id = f"{FRAME_PREFIX}{int(marker_id)}"
            tf_msg.transform.translation.x = float(tvec[0][0])
            tf_msg.transform.translation.y = float(tvec[0][1])
            tf_msg.transform.translation.z = float(tvec[0][2])
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

            with self._marker_lock:
                self.marker_position = np.array(tvec[0], dtype=np.float64)
                self.marker_rotation = np.array(r_mat, dtype=np.float64)

    def _crop_points_to_marker_roi(self, points_np, cloud_frame_id):
        """Filtert Punkte strikt auf ROI im ArUco-Marker-Frame."""
        with self._marker_lock:
            marker_position = None if self.marker_position is None else self.marker_position.copy()
            marker_rotation = None if self.marker_rotation is None else self.marker_rotation.copy()

        if marker_position is None or marker_rotation is None:
            return None

        if cloud_frame_id and self.camera_frame and cloud_frame_id != self.camera_frame and not self._frame_warned:
            self._frame_warned = True
            self.get_logger().warn(
                f"Frame-Mismatch: PointCloud in '{cloud_frame_id}', ArUco in '{self.camera_frame}'. "
                "Es wird trotzdem dieselbe Transformation verwendet."
            )

        x_min, x_max, y_min, y_max, z_min, z_max = CROP_BOUNDS_MARKER
        points_relative = points_np - marker_position
        points_marker = points_relative @ marker_rotation.T

        mask = (
            (points_marker[:, 0] >= x_min)
            & (points_marker[:, 0] <= x_max)
            & (points_marker[:, 1] >= y_min)
            & (points_marker[:, 1] <= y_max)
            & (points_marker[:, 2] >= z_min)
            & (points_marker[:, 2] <= z_max)
        )

        cropped = points_np[mask]
        if len(cropped) < MIN_CLUSTER_POINTS:
            return None
        return cropped

    def listener_callback(self, msg):
        now = time.time()
        if now - self.last_match_time < MATCH_INTERVAL:
            return

        try:
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])

            if len(points) < 150:
                return

            points_np = np.asarray(points, dtype=np.float64)
            if len(points_np) > MAX_RAW_POINTS:
                step = max(1, len(points_np) // MAX_RAW_POINTS)
                points_np = points_np[::step]

            # Harter Vor-Crop im Marker-Frame: nur ROI wird gematcht
            roi_points = self._crop_points_to_marker_roi(
                points_np,
                msg.header.frame_id or "",
            )
            if roi_points is None:
                self.last_match_time = now
                return

            scene_pcd = o3d.geometry.PointCloud()
            scene_pcd.points = o3d.utility.Vector3dVector(roi_points)

            pose = self._estimate_pose(scene_pcd)
            self.last_match_time = now
            if pose is None:
                return

            trans, rot, fitness, rmse = pose
            if self.last_translation is not None:
                trans = POSE_SMOOTH_ALPHA * trans + (1.0 - POSE_SMOOTH_ALPHA) * self.last_translation

            # Sehr leichte Rotations-Sanftheit: Vorzeichenstabilisierung
            if self.last_rotation is not None and np.trace(rot.T @ self.last_rotation) < 0:
                rot[:, 0] *= -1.0
                rot[:, 1] *= -1.0

            self.last_translation = trans
            self.last_rotation = rot
            self._publish_tf(trans, rot, msg.header)
            self.get_logger().info(
                f"Match: cropv1_clean | Fitness={fitness:.3f} | RMSE={rmse:.4f} m | "
                f"Pos=({trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f})"
            )

        except Exception as e:
            self.get_logger().error(f"Matching-Fehler: {e}")

    def _estimate_pose(self, scene_pcd):
        # 1) Tisch entfernen
        if len(scene_pcd.points) > 120:
            try:
                _, inliers = scene_pcd.segment_plane(
                    distance_threshold=RANSAC_DISTANCE_THRESHOLD,
                    ransac_n=RANSAC_N,
                    num_iterations=RANSAC_ITERATIONS,
                )
                objects = scene_pcd.select_by_index(inliers, invert=True)
            except Exception:
                objects = scene_pcd
        else:
            objects = scene_pcd

        if len(objects.points) < MIN_CLUSTER_POINTS:
            return None

        # 2) Downsample
        objects = objects.voxel_down_sample(VOXEL_SIZE)
        if len(objects.points) < MIN_CLUSTER_POINTS:
            return None

        # 3) Cluster (groesster gewinnt)
        labels = np.array(objects.cluster_dbscan(
            eps=CLUSTER_EPS,
            min_points=CLUSTER_MIN_POINTS,
            print_progress=False,
        ))

        if len(labels) == 0 or labels.max() < 0:
            return None

        cluster_id = int(np.argmax([np.sum(labels == i) for i in range(labels.max() + 1)]))
        idx = np.where(labels == cluster_id)[0]
        cluster = objects.select_by_index(idx.tolist())

        if len(cluster.points) < MIN_CLUSTER_POINTS:
            return None

        pts = np.asarray(cluster.points, dtype=np.float64)

        # 4) Initialisierung ueber PCA
        r_init = rotation_from_pca(pts)
        t_init = pts.mean(axis=0)

        init = np.eye(4)
        init[:3, :3] = r_init
        init[:3, 3] = t_init

        template_transformed = (r_init @ self.template_points.T).T + t_init

        # 5) ICP
        t_icp, fitness, rmse = icp_point_to_point(
            template_transformed,
            pts,
            max_corr_dist=ICP_MAX_CORR_DIST,
            max_iterations=ICP_MAX_ITER,
        )

        if fitness < MIN_FITNESS:
            return None

        transform = t_icp @ init
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
        return translation, rotation, fitness, rmse

    def _publish_tf(self, translation, rotation, header):
        qx, qy, qz, qw = quat_from_rotmat(rotation)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = header.stamp
        tf_msg.header.frame_id = header.frame_id or "camera_depth_optical_frame"
        tf_msg.child_frame_id = "detected_cropv1_center"
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LightweightPliersPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
