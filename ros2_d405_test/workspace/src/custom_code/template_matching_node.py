#!/usr/bin/env python3
"""
Template Matching Node für Zangen-Erkennung.

Abonniert die Live-Punktwolke der RealSense D405, entfernt die Tischebene
(RANSAC), und führt ICP-basiertes Template Matching gegen vorbereitete
Zangen-Templates durch. Die erkannte Pose wird als TF-Frame publiziert,
sodass sie in RViz2 visualisiert werden kann.

Workflow:
  1. Live-Punktwolke empfangen
  2. Tischebene per RANSAC entfernen
  3. Voxel-Downsampling
  4. Für jedes Template: Global Registration (FPFH) → Fine Registration (ICP)
  5. Bestes Match auswählen (niedrigster Fitness-Score)
  6. Pose als TF-Frame publizieren

Verwendung:
  python3 template_matching_node.py
"""
import os
import glob
import time

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPEN3D_CPU_THREAD_COUNT", "1")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import open3d as o3d
import numpy as np

# Konfiguration
POINTCLOUD_TOPIC = "/camera/camera/depth/color/points"
TEMPLATE_DIR = "/workspace/src/custom_packages/custom_code/templates"

# RANSAC Parameter für Tischebenen-Erkennung
RANSAC_DISTANCE_THRESHOLD = 0.012  # 12mm - lockerer, weniger weg filtern
RANSAC_N_POINTS = 3                # Min. Punkte für Ebenen-Schätzung
RANSAC_ITERATIONS = 1000           # RANSAC Iterationen

# ICP Parameter
VOXEL_SIZE = 0.002        # 2mm Voxel für Downsampling
ICP_THRESHOLD = 0.010     # 10mm max Korrespondenz-Distanz (erhöht von 5mm)
ICP_MAX_ITERATION = 15    # Max ICP Iterationen
MIN_DOWNSAMPLED_POINTS = 80
MAX_SCENE_POINTS = 4000
MAX_TEMPLATE_POINTS = 2500
ICP_TOL = 1e-5
NN_CHUNK_SIZE = 256

# Matching Schwellwert
MIN_FITNESS = 0.20        # noch lockerer für erste Erkennung
MIN_FITNESS_MARGIN = 0.20 # 20% Unterschied nötig um zu unterscheiden
SWITCH_CONFIRMATIONS = 3  # Anzahl aufeinanderfolgender Matches vor Template-Wechsel
POSE_SMOOTH_ALPHA = 0.35  # Translation-Glättung (0..1), höher = reaktiver

# Rate-Limiting (nicht jeden Frame matchen)
MATCH_INTERVAL = 1.0      # Sekunden zwischen Matching-Versuchen


class TemplateMatcher:
    """Enthält ein geladenes Template mit vorberechneten Features."""

    def __init__(self, name, pcd, voxel_size):
        self.name = name
        self.pcd_original = pcd

        # Downsampling
        self.pcd_down = pcd.voxel_down_sample(voxel_size)
        if len(self.pcd_down.points) < 30:
            self.pcd_down = pcd
        self.points = np.asarray(self.pcd_down.points, dtype=np.float64)
        if len(self.points) > MAX_TEMPLATE_POINTS:
            step = max(1, len(self.points) // MAX_TEMPLATE_POINTS)
            self.points = self.points[::step]

        print(f"  Template '{name}': {len(pcd.points)} Punkte, "
              f"downsampled: {len(self.pcd_down.points)}, genutzt: {len(self.points)}")


def remove_table_plane(pcd, distance_threshold, n_points, iterations):
    """
    Entfernt die größte Ebene (Tisch) aus der Punktwolke per RANSAC.

    Returns:
        objects_pcd: Punktwolke ohne Tisch
        plane_model: [a, b, c, d] der Ebenengleichung
    """
    if len(pcd.points) < 100:
        return pcd, None

    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=n_points,
        num_iterations=iterations)

    objects_pcd = pcd.select_by_index(inliers, invert=True)
    return objects_pcd, plane_model


def _rigid_transform_svd(source, target):
    """Berechnet starre Transformation (R,t) per SVD für korrespondierende Punkte."""
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

    t_vec = target_centroid - r_mat @ source_centroid
    return r_mat, t_vec


def nearest_neighbors_numpy(source_points, target_points, chunk_size=NN_CHUNK_SIZE):
    """Nächster Nachbar pro Source-Punkt via chunked NumPy (ohne SciPy)."""
    n_source = len(source_points)
    idx_out = np.empty(n_source, dtype=np.int64)
    dist_out = np.empty(n_source, dtype=np.float64)

    target_sq = np.sum(target_points * target_points, axis=1)

    for start in range(0, n_source, chunk_size):
        end = min(start + chunk_size, n_source)
        src_chunk = source_points[start:end]
        src_sq = np.sum(src_chunk * src_chunk, axis=1, keepdims=True)

        # d^2 = ||a||^2 + ||b||^2 - 2 a·b
        d2 = src_sq + target_sq[None, :] - 2.0 * (src_chunk @ target_points.T)
        np.maximum(d2, 0.0, out=d2)

        local_idx = np.argmin(d2, axis=1)
        idx_out[start:end] = local_idx
        dist_out[start:end] = np.sqrt(d2[np.arange(end - start), local_idx])

    return dist_out, idx_out


def icp_point_to_point(source_points, target_points, max_corr_dist, max_iterations, tol):
    """Robustes ICP (Point-to-Point) mit reiner NumPy-NN-Suche."""
    if len(source_points) < 30 or len(target_points) < 30:
        return np.eye(4), 0.0, float("inf")

    src = source_points.copy()
    t_total = np.eye(4)

    # Grobe Initialisierung über Zentroiden
    shift = target_points.mean(axis=0) - src.mean(axis=0)
    src = src + shift
    t_total[:3, 3] = shift

    prev_rmse = float("inf")
    rmse = float("inf")
    fitness = 0.0

    for _ in range(max_iterations):
        dists, indices = nearest_neighbors_numpy(src, target_points)
        valid = dists < max_corr_dist
        valid_count = int(valid.sum())
        if valid_count < 30:
            break

        src_corr = src[valid]
        tgt_corr = target_points[indices[valid]]

        r_mat, t_vec = _rigid_transform_svd(src_corr, tgt_corr)
        src = (r_mat @ src.T).T + t_vec

        step = np.eye(4)
        step[:3, :3] = r_mat
        step[:3, 3] = t_vec
        t_total = step @ t_total

        rmse = float(np.sqrt(np.mean(dists[valid] ** 2)))
        fitness = valid_count / float(len(source_points))

        if abs(prev_rmse - rmse) < tol:
            break
        prev_rmse = rmse

    return t_total, fitness, rmse


def transformation_to_tf(transform_matrix, parent_frame, child_frame, stamp):
    """Konvertiert eine 4x4 Transformationsmatrix in eine ROS2 TransformStamped."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    # Translation
    t.transform.translation.x = float(transform_matrix[0, 3])
    t.transform.translation.y = float(transform_matrix[1, 3])
    t.transform.translation.z = float(transform_matrix[2, 3])

    # Rotation (Matrix → Quaternion)
    rotation = transform_matrix[:3, :3]
    # Quaternion aus Rotationsmatrix berechnen
    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation[2, 1] - rotation[1, 2]) * s
        y = (rotation[0, 2] - rotation[2, 0]) * s
        z = (rotation[1, 0] - rotation[0, 1]) * s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s

    t.transform.rotation.x = float(x)
    t.transform.rotation.y = float(y)
    t.transform.rotation.z = float(z)
    t.transform.rotation.w = float(w)

    return t


class TemplateMatchingNode(Node):
    def __init__(self):
        super().__init__('template_matching_node')

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Templates laden
        self.templates = []
        self._load_templates()

        if not self.templates:
            self.get_logger().error(f"Keine Templates gefunden in {TEMPLATE_DIR}!")
            self.get_logger().error("Erstelle zuerst Templates mit pointcloud_snapshot_node.py und preprocess_template.py")
            return

        # QoS für RealSense
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            POINTCLOUD_TOPIC,
            self.listener_callback,
            qos)

        self.last_match_time = 0.0
        self.latest_points = None
        self.latest_colors = None
        self.has_rgb = None
        self.active_template_name = None
        self.candidate_template_name = None
        self.candidate_count = 0
        self.last_published_transform = None

        self.get_logger().info("Template Matching Node gestartet")
        self.get_logger().info(f"Topic: {POINTCLOUD_TOPIC}")
        self.get_logger().info(f"Templates: {[t.name for t in self.templates]}")
        self.get_logger().info(f"Matching alle {MATCH_INTERVAL}s")

    def _load_templates(self):
        """Lädt alle .pcd Templates aus dem Template-Ordner."""
        if not os.path.exists(TEMPLATE_DIR):
            self.get_logger().warn(f"Template-Ordner existiert nicht: {TEMPLATE_DIR}")
            return

        pcd_files = sorted(glob.glob(os.path.join(TEMPLATE_DIR, "*.pcd")))
        self.get_logger().info(f"Lade {len(pcd_files)} Templates aus {TEMPLATE_DIR}...")

        for path in pcd_files:
            pcd = o3d.io.read_point_cloud(path)
            if len(pcd.points) == 0:
                self.get_logger().warn(f"  Überspringe leeres Template: {path}")
                continue

            name = os.path.splitext(os.path.basename(path))[0]
            template = TemplateMatcher(name, pcd, VOXEL_SIZE)
            self.templates.append(template)

    def listener_callback(self, msg):
        """Empfängt Punktwolken und führt bei Bedarf Matching durch."""
        now = time.time()
        if now - self.last_match_time < MATCH_INTERVAL:
            return

        # Felder prüfen
        if self.has_rgb is None:
            field_names = [f.name for f in msg.fields]
            self.has_rgb = 'rgb' in field_names

        try:
            # Punkte extrahieren
            if self.has_rgb:
                fields = ("x", "y", "z", "rgb")
            else:
                fields = ("x", "y", "z")

            points_list = []
            for point in pc2.read_points(msg, skip_nans=True, field_names=fields):
                points_list.append([point[0], point[1], point[2]])

            if len(points_list) < 100:
                return

            points = np.array(points_list, dtype=np.float64)

            # Open3D Punktwolke erstellen
            scene_pcd = o3d.geometry.PointCloud()
            scene_pcd.points = o3d.utility.Vector3dVector(points)

            # Matching durchführen
            self.last_match_time = now
            self._match_templates(scene_pcd, msg.header)

        except Exception as e:
            self.get_logger().error(f"Fehler: {e}")

    def _match_templates(self, scene_pcd, header):
        """Führt Template Matching auf der Szene durch."""

        # 1. Tischebene entfernen
        objects_pcd, plane = remove_table_plane(
            scene_pcd, RANSAC_DISTANCE_THRESHOLD, RANSAC_N_POINTS, RANSAC_ITERATIONS)

        if plane is not None:
            self.get_logger().info(
                f"Tisch entfernt: {len(scene_pcd.points)} → {len(objects_pcd.points)} Punkte")
        else:
            self.get_logger().info(f"Ebene nicht erkannt, {len(scene_pcd.points)} Punkte verwendet")

        if len(objects_pcd.points) < 50:
            self.get_logger().debug("Zu wenige Punkte nach Tisch-Entfernung")
            return

        # 2. Downsampling
        scene_down = objects_pcd.voxel_down_sample(VOXEL_SIZE)
        if len(scene_down.points) < MIN_DOWNSAMPLED_POINTS:
            self.get_logger().debug("Zu wenige Punkte nach Downsampling")
            return

        scene_points = np.asarray(scene_down.points, dtype=np.float64)
        if len(scene_points) > MAX_SCENE_POINTS:
            step = max(1, len(scene_points) // MAX_SCENE_POINTS)
            scene_points = scene_points[::step]

        # 4. Gegen jedes Template matchen
        best_transform = None
        best_template = None
        best_fitness = 0.0
        best_rmse = float("inf")
        second_best_fitness = 0.0

        for template in self.templates:
            try:
                transform, fitness, rmse = icp_point_to_point(
                    template.points,
                    scene_points,
                    max_corr_dist=ICP_THRESHOLD,
                    max_iterations=ICP_MAX_ITERATION,
                    tol=ICP_TOL,
                )

                if (fitness > best_fitness) or (abs(fitness - best_fitness) < 1e-6 and rmse < best_rmse):
                    second_best_fitness = best_fitness
                    best_fitness = fitness
                    best_rmse = rmse
                    best_transform = transform
                    best_template = template
                elif fitness > second_best_fitness:
                    second_best_fitness = fitness

            except Exception as e:
                self.get_logger().debug(f"Matching-Fehler für {template.name}: {e}")

        # 5. Ergebnis publizieren
        if best_transform is not None and best_fitness >= MIN_FITNESS:
            self.get_logger().info(
                f"Kandidat: {best_template.name} | Fitness: {best_fitness:.3f} | "
                f"2nd: {second_best_fitness:.3f} | RMSE: {best_rmse:.4f}m")

            # Template-Wechsel erst nach mehreren Bestätigungen (Anti-Flattern)
            # Erste Erkennung (active=None) ist sofort
            if self.active_template_name != best_template.name:
                # Ambiguity Gate ONLY bei Wechsel
                # Nur blockieren wenn wir schon ein Template haben (nicht beim Start)
                if self.active_template_name is not None and (best_fitness - second_best_fitness) < MIN_FITNESS_MARGIN:
                    self.get_logger().info(
                        f"  → Ambiguous für Wechsel (Abstand {best_fitness - second_best_fitness:.3f} < {MIN_FITNESS_MARGIN}), übersprungen")
                    return
                if self.active_template_name is not None:
                    # Wechsel von einem bekannten Template → Confirmations nötig
                    if self.candidate_template_name == best_template.name:
                        self.candidate_count += 1
                    else:
                        self.candidate_template_name = best_template.name
                        self.candidate_count = 1

                    if self.candidate_count < SWITCH_CONFIRMATIONS:
                        self.get_logger().info(
                            f"  → Wechsel pending: {best_template.name} "
                            f"({self.candidate_count}/{SWITCH_CONFIRMATIONS})")
                        return

                self.active_template_name = best_template.name
                self.candidate_template_name = None
                self.candidate_count = 0
                self.last_published_transform = None
                self.get_logger().info(f"  → Template aktiv: {self.active_template_name}")
            else:
                # Gleiches Template → einfach weitermachen
                pass

            transform = best_transform

            # Translation glätten für stabilere Pose in RViz
            if self.last_published_transform is not None:
                smooth_transform = transform.copy()
                smooth_transform[:3, 3] = (
                    POSE_SMOOTH_ALPHA * transform[:3, 3] +
                    (1.0 - POSE_SMOOTH_ALPHA) * self.last_published_transform[:3, 3]
                )
                transform = smooth_transform

            self.last_published_transform = transform.copy()

            self.get_logger().info(
                f"Match: {best_template.name} | "
                f"Fitness: {best_fitness:.3f} | "
                f"RMSE: {best_rmse:.4f}m")

            # TF publizieren
            tf_msg = transformation_to_tf(
                transform,
                parent_frame=header.frame_id or "camera_depth_optical_frame",
                child_frame=f"detected_{best_template.name}",
                stamp=header.stamp)

            self.tf_broadcaster.sendTransform(tf_msg)

            # Position loggen
            pos = transform[:3, 3]
            self.get_logger().info(
                f"Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f} m")
        else:
            self.get_logger().info(
                f"Kein Match: beste Fitness={best_fitness:.3f} (Schwellwert={MIN_FITNESS})")


def main(args=None):
    rclpy.init(args=args)
    node = TemplateMatchingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
