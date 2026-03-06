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
import struct
import glob
import time

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
RANSAC_DISTANCE_THRESHOLD = 0.005  # 5mm - Punkte innerhalb dieses Abstands gehören zur Ebene
RANSAC_N_POINTS = 3                # Min. Punkte für Ebenen-Schätzung
RANSAC_ITERATIONS = 1000           # RANSAC Iterationen

# ICP Parameter
VOXEL_SIZE = 0.002        # 2mm Voxel für Downsampling
ICP_THRESHOLD = 0.005     # 5mm max Korrespondenz-Distanz
ICP_MAX_ITERATION = 50    # Max ICP Iterationen

# Matching Schwellwert
MIN_FITNESS = 0.3         # Mindest-Fitness (0-1) damit ein Match akzeptiert wird

# Rate-Limiting (nicht jeden Frame matchen)
MATCH_INTERVAL = 0.5      # Sekunden zwischen Matching-Versuchen


class TemplateMatcher:
    """Enthält ein geladenes Template mit vorberechneten Features."""

    def __init__(self, name, pcd, voxel_size):
        self.name = name
        self.pcd_original = pcd

        # Downsampling
        self.pcd_down = pcd.voxel_down_sample(voxel_size)

        # Normalen berechnen
        self.pcd_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 2, max_nn=30))

        # FPFH-Features für Global Registration
        self.fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            self.pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 5, max_nn=100))

        print(f"  Template '{name}': {len(pcd.points)} Punkte, "
              f"downsampled: {len(self.pcd_down.points)}")


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


def global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    """
    Grobe Ausrichtung per FPFH-Feature Matching (RANSAC-basiert).
    Findet eine initiale Transformation, auch wenn die Objekte weit auseinander liegen.
    """
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down,
        source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=voxel_size * 2.5,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 2.5)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def refine_icp(source, target, init_transform, threshold, max_iteration):
    """
    Feine Ausrichtung per Point-to-Plane ICP.
    Verfeinert die Transformation aus der Global Registration.
    """
    # Normalen für Point-to-Plane ICP
    target.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30))

    result = o3d.pipelines.registration.registration_icp(
        source, target,
        threshold,
        init_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration))
    return result


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
            self.get_logger().debug(
                f"Tisch entfernt: {len(scene_pcd.points)} → {len(objects_pcd.points)} Punkte")

        if len(objects_pcd.points) < 50:
            self.get_logger().debug("Zu wenige Punkte nach Tisch-Entfernung")
            return

        # 2. Downsampling
        scene_down = objects_pcd.voxel_down_sample(VOXEL_SIZE)

        # 3. Normalen & Features für die Szene berechnen
        scene_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=VOXEL_SIZE * 2, max_nn=30))

        scene_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            scene_down,
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=VOXEL_SIZE * 5, max_nn=100))

        # 4. Gegen jedes Template matchen
        best_result = None
        best_template = None
        best_fitness = 0.0

        for template in self.templates:
            try:
                # Global Registration (grobe Ausrichtung)
                global_reg = global_registration(
                    template.pcd_down, scene_down,
                    template.fpfh, scene_fpfh,
                    VOXEL_SIZE)

                # ICP Refinement (feine Ausrichtung)
                icp_result = refine_icp(
                    template.pcd_down, scene_down,
                    global_reg.transformation,
                    ICP_THRESHOLD, ICP_MAX_ITERATION)

                if icp_result.fitness > best_fitness:
                    best_fitness = icp_result.fitness
                    best_result = icp_result
                    best_template = template

            except Exception as e:
                self.get_logger().debug(f"Matching-Fehler für {template.name}: {e}")

        # 5. Ergebnis publizieren
        if best_result is not None and best_fitness >= MIN_FITNESS:
            transform = best_result.transformation

            self.get_logger().info(
                f"Match: {best_template.name} | "
                f"Fitness: {best_fitness:.3f} | "
                f"RMSE: {best_result.inlier_rmse:.4f}m")

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
            self.get_logger().debug(
                f"Kein Match (beste Fitness: {best_fitness:.3f}, Schwellwert: {MIN_FITNESS})")


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
