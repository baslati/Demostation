#!/usr/bin/env python3
"""
Monolithischer D405 Template-Scanner.

Workflow:
  1. ArUco-Marker erkennen (Hintergrund-Thread)
  2. Enter → Snapshot der aktuellen Szene
  3. Open3D: vollständige Szene anzeigen
  4. Crop-Bounds relativ zum Marker eingeben (6 Zahlen)
  5. Open3D: gecropte Cloud im Marker-Frame anzeigen (Achsen sichtbar)
  6. Preprocessing: Cluster → Outlier Removal → Voxel → Zentrierung → Y-Ausrichtung
  7. Open3D: fertiges Template anzeigen
  8. Name eingeben → speichern als templates/<name>_clean_direction.pcd
"""

import os
import sys
import threading
import time

import cv2
import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


# ── Konfiguration ──────────────────────────────────────────────────────────────
MARKER_ID       = 0
MARKER_SIZE_M   = 0.05
VOXEL_SIZE      = 0.0025   # muss mit VOXEL_SIZE_TEMPLATE im Detection-Node übereinstimmen
CLUSTER_EPS     = 0.010    # DBSCAN: max. Abstand zwischen Punkten (m)
CLUSTER_MIN_PTS = 10
OUTLIER_NB      = 20       # Statistical Outlier: Nachbarn
OUTLIER_STD     = 2.0      # Statistical Outlier: Standardabweichungs-Schwellwert
TEMPLATE_DIR    = "/workspace/src/custom_packages/custom_code/templates"

TOPICS = {
    "image": "/camera/camera/color/image_raw",
    "info":  "/camera/camera/color/camera_info",
    "cloud": "/camera/camera/depth/color/points",
}
# ──────────────────────────────────────────────────────────────────────────────


class ScannerNode(Node):
    def __init__(self):
        super().__init__("d405_template_scanner")
        self._lock = threading.Lock()

        self.k = None
        self.dist = None
        self.latest_cloud = None   # raw PointCloud2
        self.marker_pos = None     # np [3], Kamera-Frame
        self.marker_rot = None     # np 3×3 Rotationsmatrix

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector   = cv2.aruco.ArucoDetector(
            self.aruco_dict, cv2.aruco.DetectorParameters()
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CameraInfo, TOPICS["info"],  self._on_info,  qos)
        self.create_subscription(Image,      TOPICS["image"], self._on_image, qos)
        self.create_subscription(PointCloud2, TOPICS["cloud"], self._on_cloud, qos)

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _on_info(self, msg):
        with self._lock:
            if self.k is None:
                self.k    = np.array(msg.k, dtype=np.float64).reshape(3, 3)
                self.dist = np.array(msg.d, dtype=np.float64)

    def _on_image(self, msg):
        with self._lock:
            if self.k is None:
                return
            k, dist = self.k.copy(), self.dist.copy()

        frame = _ros_image_to_bgr(msg)
        if frame is None:
            return

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return

        for mid, corner in zip(ids.flatten(), corners):
            if int(mid) != MARKER_ID:
                continue
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corner], MARKER_SIZE_M, k, dist
            )
            rmat, _ = cv2.Rodrigues(rvecs[0][0])
            with self._lock:
                self.marker_pos = tvecs[0][0].copy()
                self.marker_rot = rmat.copy()

    def _on_cloud(self, msg):
        with self._lock:
            self.latest_cloud = msg

    # ── API ────────────────────────────────────────────────────────────────────

    def status(self):
        """Gibt (hat_info, hat_marker, hat_cloud) zurück."""
        with self._lock:
            return (
                self.k is not None,
                self.marker_pos is not None,
                self.latest_cloud is not None,
            )

    def snapshot(self):
        """
        Gibt (points_xyz, marker_pos, marker_rot) zurück.
        Alle Punkte im Kamera-Frame als np.float64.
        """
        with self._lock:
            if self.latest_cloud is None or self.marker_pos is None:
                return None, None, None
            msg = self.latest_cloud
            pos = self.marker_pos.copy()
            rot = self.marker_rot.copy()

        pts = _cloud_to_xyz(msg)
        if pts is None or len(pts) == 0:
            return None, None, None
        return pts, pos, rot


# ── Hilfsfunktionen ────────────────────────────────────────────────────────────

def _ros_image_to_bgr(msg):
    try:
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.encoding == "rgb8":
            img = data.reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if msg.encoding == "bgr8":
            return data.reshape(msg.height, msg.width, 3)
    except Exception:
        pass
    return None


def _cloud_to_xyz(msg):
    try:
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float64)
        return pts if len(pts) > 0 else None
    except Exception:
        return None


def crop_in_marker_frame(points, marker_pos, marker_rot, bounds):
    """
    Transformiert Punkte in den Marker-Frame und croppt.
    Gibt (cam_pts, marker_pts) zurück.
    """
    pts_marker = (points - marker_pos) @ marker_rot.T
    x0, x1, y0, y1, z0, z1 = bounds
    mask = (
        (pts_marker[:, 0] >= x0) & (pts_marker[:, 0] <= x1) &
        (pts_marker[:, 1] >= y0) & (pts_marker[:, 1] <= y1) &
        (pts_marker[:, 2] >= z0) & (pts_marker[:, 2] <= z1)
    )
    return points[mask], pts_marker[mask]


def _color_by_z(points):
    """Färbt Punkte nach Höhe: blau (niedrig) → rot (hoch)."""
    z = points[:, 2]
    lo, hi = z.min(), z.max()
    t = (z - lo) / (hi - lo) if hi > lo else np.full(len(z), 0.5)
    return np.column_stack([t, np.full(len(t), 0.3), 1.0 - t])


def show_pcd(points, title, with_axes=True, axes_size=0.05):
    """Zeigt Punktwolke in Open3D. Blockiert bis Fenster geschlossen wird."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(_color_by_z(points))

    geometries = [pcd]
    if with_axes:
        geometries.append(
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=axes_size)
        )

    print(f"\n[Open3D] {title}  ({len(points)} Punkte)  — Fenster schließen zum Fortfahren")
    o3d.visualization.draw_geometries(
        geometries,
        window_name=f"{title}  ({len(points)} Punkte)",
        width=1024,
        height=768,
    )


# ── Preprocessing ──────────────────────────────────────────────────────────────

def _largest_cluster(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    labels = np.array(
        pcd.cluster_dbscan(eps=CLUSTER_EPS, min_points=CLUSTER_MIN_PTS, print_progress=False)
    )
    if labels.max() < 0:
        print("  Cluster: keiner gefunden, alle Punkte behalten")
        return points
    n = labels.max() + 1
    sizes = [int(np.sum(labels == i)) for i in range(n)]
    best = int(np.argmax(sizes))
    result = pcd.select_by_index(np.where(labels == best)[0].tolist())
    print(f"  Cluster: {n} gefunden → größter ({best}): {len(result.points)} Punkte")
    return np.asarray(result.points, dtype=np.float64)


def _remove_outliers(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if len(pcd.points) <= OUTLIER_NB:
        return points
    filtered, _ = pcd.remove_statistical_outlier(
        nb_neighbors=OUTLIER_NB, std_ratio=OUTLIER_STD
    )
    print(f"  Outlier: {len(pcd.points) - len(filtered.points)} entfernt → {len(filtered.points)} Punkte")
    return np.asarray(filtered.points, dtype=np.float64)


def _voxel_downsample(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd = pcd.voxel_down_sample(VOXEL_SIZE)
    print(f"  Voxel ({VOXEL_SIZE * 1000:.1f}mm): {len(pcd.points)} Punkte")
    return np.asarray(pcd.points, dtype=np.float64)


def _align_y_to_tool(points):
    """Längste PCA-Achse auf +Y ausrichten (Zangenachse)."""
    centered = points - points.mean(axis=0)
    cov = centered.T @ centered / max(1, len(points) - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    basis = eigvecs[:, np.argsort(eigvals)[::-1]]
    if np.linalg.det(basis) < 0.0:
        basis[:, 2] *= -1.0

    y_axis = basis[:, 0]
    if y_axis[1] < 0.0:
        y_axis = -y_axis

    z_axis = np.cross(basis[:, 1], y_axis)
    z_norm = np.linalg.norm(z_axis)
    if z_norm < 1e-9:
        print("  Y-Ausrichtung: PCA degeneriert, übersprungen")
        return points
    z_axis /= z_norm
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)

    rot = np.column_stack((x_axis, y_axis, z_axis)).T
    print("  Y-Achse: entlang Zangenachse (PCA)")
    return points @ rot.T


def preprocess(points):
    print(f"\n[Preprocessing]  Input: {len(points)} Punkte")
    points = _largest_cluster(points)
    points = _remove_outliers(points)
    points = _voxel_downsample(points)
    centroid = points.mean(axis=0)
    points -= centroid
    print(f"  Centroid: [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}] m")
    points = _align_y_to_tool(points)
    print(f"  Output:   {len(points)} Punkte")
    return points


def save_template(points, name):
    os.makedirs(TEMPLATE_DIR, exist_ok=True)
    filename = name if name.endswith(".pcd") else f"{name}_clean_direction.pcd"
    path = os.path.join(TEMPLATE_DIR, filename)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if o3d.io.write_point_cloud(path, pcd):
        print(f"  Gespeichert: {path}  ({len(points)} Punkte)")
        return True
    print(f"  FEHLER: Konnte nicht speichern: {path}")
    return False


# ── Hauptprogramm ──────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ScannerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    os.makedirs(TEMPLATE_DIR, exist_ok=True)
    print("=" * 60)
    print("  D405 Template Scanner")
    print(f"  Marker ID: {MARKER_ID}  |  Größe: {MARKER_SIZE_M * 100:.0f} cm")
    print(f"  Voxel-Größe: {VOXEL_SIZE * 1000:.1f} mm")
    print(f"  Speicherort: {TEMPLATE_DIR}")
    print("=" * 60)

    try:
        while True:
            # ── Warten auf Kamera, Marker und Cloud ───────────────────────────
            print("\nWarte auf Kamera-Info, ArUco-Marker und Pointcloud ...")
            while True:
                has_info, has_marker, has_cloud = node.status()
                if has_info and has_marker and has_cloud:
                    break
                missing = []
                if not has_info:   missing.append("Kamera-Info")
                if not has_marker: missing.append(f"Marker ID {MARKER_ID}")
                if not has_cloud:  missing.append("Pointcloud")
                print(f"  Fehlend: {', '.join(missing)}", end="\r", flush=True)
                time.sleep(0.3)
            print("  Alles bereit.              ")

            input("\nEnter → Snapshot aufnehmen ...")

            # ── Snapshot ──────────────────────────────────────────────────────
            pts, marker_pos, marker_rot = node.snapshot()
            if pts is None:
                print("Snapshot fehlgeschlagen (keine Cloud oder kein Marker). Erneut versuchen.")
                continue
            print(f"Snapshot: {len(pts)} Punkte")
            show_pcd(pts, "Snapshot — vollständige Szene", with_axes=False)

            # ── Crop-Schleife ─────────────────────────────────────────────────
            cropped_marker = None
            while True:
                print("\nCrop-Bounds relativ zum Marker eingeben (6 Zahlen, Meter):")
                print("  Format : x_min x_max  y_min y_max  z_min z_max")
                print("  Achsen : +X=Rot  +Y=Grün  +Z=Blau  (ArUco-Frame)")
                print("  Beispiel: -0.03 0.38 -0.22 0.03 0.008 0.03")

                line = input("> ").strip()
                try:
                    vals = [float(v) for v in line.split()]
                except ValueError:
                    print("Ungültige Eingabe — nur Zahlen eingeben.")
                    continue
                if len(vals) != 6:
                    print(f"Erwartet 6 Zahlen, erhalten {len(vals)}.")
                    continue

                _, cropped_marker = crop_in_marker_frame(pts, marker_pos, marker_rot, tuple(vals))
                if len(cropped_marker) == 0:
                    print("Keine Punkte in diesem Bereich! Andere Bounds eingeben.")
                    continue

                print(f"Gecroppt: {len(cropped_marker)} Punkte")
                show_pcd(cropped_marker, "Gecropte Cloud — Marker-Frame", with_axes=True)

                if input("Crop OK? (j/n) > ").strip().lower() == "j":
                    break

            # ── Preprocessing ─────────────────────────────────────────────────
            template_pts = preprocess(cropped_marker)
            show_pcd(
                template_pts,
                "Fertiges Template — zentriert, Y = Zangenachse",
                with_axes=True,
                axes_size=0.02,
            )

            if input("Template speichern? (j/n) > ").strip().lower() != "j":
                print("Verworfen.")
            else:
                name = input("Template-Name (ohne _clean_direction.pcd): ").strip()
                if not name:
                    name = f"template_{int(time.time())}"
                save_template(template_pts, name)

            if input("\nNoch ein Template scannen? (j/n) > ").strip().lower() != "j":
                break

    except KeyboardInterrupt:
        print("\nAbgebrochen.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
