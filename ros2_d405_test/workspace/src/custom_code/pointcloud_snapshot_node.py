#!/usr/bin/env python3
import select
import struct
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

# Persistenter Speicherpfad (gemountetes Volume -> bleibt nach Container-Stopp erhalten)
SAVE_DIR = "/workspace/src/custom_packages/custom_code/scans"

# Topic-Name (ggf. anpassen: ros2 topic list | grep points)
POINTCLOUD_TOPIC = "/camera/camera/depth/color/points"


class PointCloudSnapshotNode(Node):
    def __init__(self):
        super().__init__('pointcloud_snapshot_node')

        # QoS-Profil kompatibel mit RealSense
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
        self.latest_points = None
        self.latest_colors = None
        self.point_count = 0
        self.received = False
        self.has_rgb = None

        # Scan-Ordner erstellen falls nicht vorhanden
        os.makedirs(SAVE_DIR, exist_ok=True)

        self.get_logger().info(f"Abonniere Topic: {POINTCLOUD_TOPIC}")
        self.get_logger().info('Snapshot Node gestartet. Warte auf Punktwolke...')
        self.get_logger().info(f"Speicherort: {SAVE_DIR}")
        self.get_logger().info("Drücke ENTER um einen Snapshot zu speichern, Ctrl+C zum Beenden.")

    def listener_callback(self, msg):
        # Prüfe beim ersten Mal ob RGB vorhanden ist
        if self.has_rgb is None:
            field_names = [f.name for f in msg.fields]
            self.has_rgb = 'rgb' in field_names
            self.get_logger().info(f"Felder in PointCloud2: {field_names}")
            self.get_logger().info(f"RGB vorhanden: {self.has_rgb}")

        try:
            if self.has_rgb:
                fields = ("x", "y", "z", "rgb")
            else:
                fields = ("x", "y", "z")

            points_list = []
            colors_list = []

            for point in pc2.read_points(msg, skip_nans=True, field_names=fields):
                points_list.append([point[0], point[1], point[2]])
                if self.has_rgb:
                    rgb_int = struct.unpack('I', struct.pack('f', point[3]))[0]
                    r = ((rgb_int >> 16) & 0xFF) / 255.0
                    g = ((rgb_int >> 8) & 0xFF) / 255.0
                    b = (rgb_int & 0xFF) / 255.0
                    colors_list.append([r, g, b])

            if len(points_list) == 0:
                return

            self.latest_points = np.array(points_list, dtype=np.float64)
            self.latest_colors = np.array(colors_list, dtype=np.float64) if colors_list else None
            self.point_count = len(points_list)

            if not self.received:
                self.received = True
                self.get_logger().info(f"Erste Punktwolke empfangen! ({self.point_count} Punkte)")

        except Exception as e:
            self.get_logger().error(f"Fehler beim Lesen der Punktwolke: {e}")

    def save_snapshot(self):
        if self.latest_points is None:
            self.get_logger().warn('Noch keine Punktwolke empfangen!')
            return

        self.get_logger().info(f"Aktuelle Wolke: {self.point_count} Punkte")
        filename = input("Dateiname (z.B. pliers_long): ").strip() or "snapshot"

        # .pcd Endung sicherstellen
        if not filename.endswith('.pcd'):
            filename += '.pcd'

        full_path = os.path.join(SAVE_DIR, filename)

        # Open3D nur zum Speichern verwenden (nicht im Callback)
        try:
            import open3d as o3d
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.latest_points)
            if self.latest_colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(self.latest_colors)
            o3d.io.write_point_cloud(full_path, pcd)
        except Exception as e:
            # Fallback: Als numpy .npy speichern falls Open3D crasht
            self.get_logger().warn(f"Open3D Fehler: {e}. Speichere als .npy statt .pcd")
            full_path = full_path.replace('.pcd', '.npy')
            np.save(full_path, self.latest_points)

        self.get_logger().info(f'Gespeichert: {full_path} ({self.point_count} Punkte)')

        # Vorhandene Scans anzeigen
        scans = [f for f in os.listdir(SAVE_DIR) if f.endswith(('.pcd', '.npy'))]
        self.get_logger().info(f"Vorhandene Scans ({len(scans)}): {', '.join(sorted(scans))}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSnapshotNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                node.save_snapshot()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()