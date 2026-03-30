#!/usr/bin/env python3
"""
Robustes Single-Template ICP Matching mit ROI im ArUco-Frame.

Ziel:
- Nur im definierten ROI suchen (relativ zu ArUco Marker)
- Auch flach auf dem Tisch liegende Zange robust erkennen
- False positives ausserhalb ROI vermeiden

Strategie:
1) ArUco Pose live schätzen
2) Punktwolke in Marker-ROI hart cropen
3) Plane-Removal versuchen, bei Bedarf auf ROI ohne Plane-Removal zurueckfallen
4) Mehrere Cluster + mehrere ICP-Initialisierungen testen
5) Bestes Ergebnis als TF publizieren
"""
import os
import threading
import time

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPEN3D_CPU_THREAD_COUNT", "1")

import cv2
import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import TransformBroadcaster


POINTCLOUD_TOPIC = "/camera/camera/depth/color/points"
IMAGE_TOPIC = "/camera/camera/color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
ROI_CLOUD_TOPIC = "/cloud_roi_cropped"

TEMPLATE_PATH = "/workspace/src/custom_packages/custom_code/templates/cropv1_clean.pcd"

MARKER_SIZE = 0.05
MARKER_ID = 0
FRAME_PREFIX = "aruco_"

# x_min x_max y_min y_max z_min z_max (im Marker-Frame)
CROP_BOUNDS_MARKER = (-0.03, 0.38, -0.22, 0.03, 0.008, 0.03)

MATCH_INTERVAL = 1.0
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

ICP_THRESHOLD = 0.008
ICP_MAX_ITER = 45
MIN_SEARCH_FITNESS = 0.80
MAX_SEARCH_RMSE = 0.0042

POSE_SMOOTH_ALPHA = 0.55

ARUCO_FILTER_ALPHA = 0.12
ARUCO_MAX_JUMP_M = 0.02
ARUCO_DEADBAND_M = 0.0015

TRACKING_MIN_FITNESS = 0.70
TRACKING_MAX_RMSE = 0.0042
REDETECT_PERIOD = 2
PUBLISH_ROI_CLOUD = True

POINT_COUNT_RATIO_MIN = 0.45
POINT_COUNT_RATIO_MAX = 1.80
PCA_EXTENT_RATIO_MIN = 0.55
PCA_EXTENT_RATIO_MAX = 1.70

DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def quat_from_rotmat(r):
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
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


class TemplateMatchingRoiIcpNode(Node):
    def __init__(self):
        super().__init__("template_matching_roi_icp_node")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.k = None
        self.dist = None
        self.camera_frame = "camera_color_optical_frame"
        self.marker_position = None
        self.marker_rotation = None
        self.marker_filtered_tvec = None
        self.marker_filtered_rvec = None
        self.marker_last_stamp = None
        self.marker_lock = threading.Lock()

        self.last_match_time = 0.0
        self.last_translation = None
        self.last_rotation = None
        self.last_transform = None
        self.match_cycle = 0

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP["DICT_4X4_50"])
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        if not self._load_template():
            self.get_logger().error("Template konnte nicht geladen werden.")
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(PointCloud2, POINTCLOUD_TOPIC, self._on_cloud, qos)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, 10)
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, 10)
        self.pub_roi_cloud = self.create_publisher(PointCloud2, ROI_CLOUD_TOPIC, 10)

        self.get_logger().info("Robustes ROI-ICP Matching gestartet")
        self.get_logger().info(f"Template: {TEMPLATE_PATH}")
        self.get_logger().info(f"Matching-Intervall: {MATCH_INTERVAL:.1f}s")
        self.get_logger().info(
            f"ArUco-Filter: alpha={ARUCO_FILTER_ALPHA:.2f}, max_jump={ARUCO_MAX_JUMP_M:.3f}m"
        )
        self.get_logger().info(
            "ROI Marker-Frame: "
            f"x[{CROP_BOUNDS_MARKER[0]:.3f},{CROP_BOUNDS_MARKER[1]:.3f}] "
            f"y[{CROP_BOUNDS_MARKER[2]:.3f},{CROP_BOUNDS_MARKER[3]:.3f}] "
            f"z[{CROP_BOUNDS_MARKER[4]:.3f},{CROP_BOUNDS_MARKER[5]:.3f}]"
        )
        self.get_logger().info(
            f"ROI-Cloud Publishing: {PUBLISH_ROI_CLOUD} auf Topic {ROI_CLOUD_TOPIC}"
        )

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

        self.get_logger().info(
            f"Template geladen: {len(self.template_points)} Punkte (zentriert)"
        )
        self.get_logger().info(
            "Template PCA-Extent: "
            f"[{self.template_pca_extent[0]:.3f}, {self.template_pca_extent[1]:.3f}, {self.template_pca_extent[2]:.3f}] m"
        )
        return True

    def _pca_extent(self, points):
        """Objektgroesse entlang der PCA-Achsen."""
        if len(points) < 10:
            return np.array([1e-6, 1e-6, 1e-6], dtype=np.float64)
        r = pca_rotation(points)
        centered = points - points.mean(axis=0)
        proj = centered @ r
        ext = np.ptp(proj, axis=0)
        return np.maximum(ext, 1e-6)

    def _on_camera_info(self, msg: CameraInfo):
        if self.k is not None:
            return
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id or self.camera_frame
        self.get_logger().info(f"CameraInfo erhalten: {self.camera_frame}")

    def _on_image(self, msg: Image):
        if self.k is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, self.k, self.dist)

        for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
            if int(marker_id) != MARKER_ID:
                continue

            raw_rvec = np.array(rvec[0], dtype=np.float64).reshape(3)
            raw_tvec = np.array(tvec[0], dtype=np.float64).reshape(3)

            # ArUco Pose glätten und Ausreißer dämpfen
            if self.marker_filtered_tvec is None:
                filt_tvec = raw_tvec
                filt_rvec = raw_rvec
            else:
                jump = np.linalg.norm(raw_tvec - self.marker_filtered_tvec)
                if jump > ARUCO_MAX_JUMP_M:
                    # Sprung verwerfen: bisherigen stabilen Wert behalten
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
                self.marker_last_stamp = time.time()

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
        now = time.time()
        if now - self.last_match_time < MATCH_INTERVAL:
            return

        try:
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])

            if len(points) < MIN_CLUSTER_POINTS:
                return

            points_np = np.asarray(points, dtype=np.float64)
            if len(points_np) > MAX_RAW_POINTS:
                step = max(1, len(points_np) // MAX_RAW_POINTS)
                points_np = points_np[::step]

            roi_points = self._crop_roi(points_np)
            if roi_points is None:
                self.last_match_time = now
                return

            if PUBLISH_ROI_CLOUD:
                self._publish_roi_cloud(msg.header, roi_points)

            self.match_cycle += 1
            mode = "search"

            # Tracking nur zwischen periodischen Neu-Suchen verwenden.
            use_tracking = self.last_transform is not None and (self.match_cycle % REDETECT_PERIOD != 0)

            pose = None
            if use_tracking:
                pose = self._track_with_last_transform(roi_points)
                if pose is not None:
                    mode = "track"

            # Regelmaessig vollstaendig neu suchen, damit Positionswechsel sicher uebernommen werden.
            if pose is None:
                pose = self._match_roi_points(roi_points)
                mode = "search"

            self.last_match_time = now
            if pose is None:
                self.last_transform = None
                self.last_translation = None
                self.last_rotation = None
                return

            translation, rotation, fitness, rmse, transform = pose

            if self.last_translation is not None:
                translation = (
                    POSE_SMOOTH_ALPHA * translation
                    + (1.0 - POSE_SMOOTH_ALPHA) * self.last_translation
                )

            self.last_translation = translation
            self.last_rotation = rotation
            self.last_transform = transform

            self._publish_detected_tf(translation, rotation, msg.header)
            self.get_logger().info(
                f"Match({mode}): cropv1_clean | Fitness={fitness:.3f} | RMSE={rmse:.4f} m | "
                f"Pos=({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})"
            )

        except Exception as e:
            self.get_logger().error(f"Matching-Fehler: {e}")

    def _publish_roi_cloud(self, header, points_np):
        """Publiziert die hart gecroppte ROI-Punktwolke (XYZ)."""
        try:
            if points_np is None or len(points_np) == 0:
                return

            # Nur XYZ publizieren fuer geringe Last.
            msg = pc2.create_cloud_xyz32(header, points_np[:, :3].tolist())
            self.pub_roi_cloud.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"ROI-Publish Fehler: {e}")

    def _cluster_candidates(self, pcd):
        labels = np.array(pcd.cluster_dbscan(
            eps=CLUSTER_EPS,
            min_points=CLUSTER_MIN_POINTS,
            print_progress=False,
        ))

        if len(labels) == 0 or labels.max() < 0:
            return [pcd]

        candidates = []
        for i in range(labels.max() + 1):
            idx = np.where(labels == i)[0]
            if len(idx) < MIN_CLUSTER_POINTS:
                continue
            c = pcd.select_by_index(idx.tolist())
            candidates.append(c)

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

        # Mehrere Startorientierungen gegen Symmetrien / flache Lage
        z_hyp = [0.0, np.pi / 2.0, np.pi, -np.pi / 2.0]

        for ang in z_hyp:
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

        # leichtes Denoising
        pcd = pcd.voxel_down_sample(VOXEL_SIZE_SCENE)
        if len(pcd.points) < MIN_CLUSTER_POINTS:
            return None

        # Plane-Removal versuchen, bei zu wenig Restpunkten fallback auf original
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

            # Plausibilitaetscheck: Punktanzahl nahe Template
            point_ratio = len(cand_points) / float(max(1, self.template_point_count))
            if point_ratio < POINT_COUNT_RATIO_MIN or point_ratio > POINT_COUNT_RATIO_MAX:
                continue

            # Plausibilitaetscheck: Objektabmessungen entlang PCA-Achsen
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

        if best_tf is None:
            return None
        if best_fit < MIN_SEARCH_FITNESS:
            return None
        if best_rmse > MAX_SEARCH_RMSE:
            return None

        rot = best_tf[:3, :3]
        trans = best_tf[:3, 3]
        return trans, rot, best_fit, best_rmse, best_tf

    def _track_with_last_transform(self, roi_points):
        """Verfolgt mit letztem gueltigen Transform weiter, um Aussetzer zu vermeiden."""
        if self.last_transform is None:
            return None

        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(self.template_points)
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(roi_points)

        result = o3d.pipelines.registration.registration_icp(
            source,
            target,
            ICP_THRESHOLD,
            self.last_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max(15, ICP_MAX_ITER // 2)),
        )

        fit = float(result.fitness)
        rmse = float(result.inlier_rmse)
        if fit < TRACKING_MIN_FITNESS or rmse > TRACKING_MAX_RMSE:
            return None

        tf = result.transformation
        rot = tf[:3, :3]
        trans = tf[:3, 3]
        return trans, rot, fit, rmse, tf

    def _publish_detected_tf(self, translation, rotation, header):
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
    node = TemplateMatchingRoiIcpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
