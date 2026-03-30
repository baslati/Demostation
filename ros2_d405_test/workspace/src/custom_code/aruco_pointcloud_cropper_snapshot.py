#!/usr/bin/env python3
"""
Erweiterung des ArUco Point Cloud Croppers:
- Nach bestaetigtem Crop wird ein Snapshot als Template gespeichert.
- Speichert bevorzugt als .pcd (Open3D), Fallback als .npy.
"""
import os
import struct
from datetime import datetime

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2

from aruco_pointcloud_cropper import ArucoPointCloudCropper, WorkflowState


class ArucoPointCloudCropperSnapshot(ArucoPointCloudCropper):
    def __init__(self):
        super().__init__()
        self.declare_parameter(
            "snapshot_save_dir",
            "/workspace/src/custom_packages/custom_code/scans",
        )
        self.save_dir = self.get_parameter("snapshot_save_dir").value
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"Snapshot-Verzeichnis: {self.save_dir}")

    def _input_loop(self):
        """Thread fuer Benutzer-Eingaben mit Auto-Snapshot nach Crop-Bestaetigung."""
        while rclpy.ok():
            try:
                user_input = input().strip()

                with self.lock:
                    if self.state == WorkflowState.PUBLISH_FULL:
                        if user_input == "":
                            if self.current_pointcloud is None:
                                print(">>> FEHLER: Noch keine Point Cloud empfangen!")
                                continue
                            if self.marker_position is None or self.marker_rotation is None:
                                print(">>> FEHLER: Noch keine Marker-Pose verfuegbar!")
                                continue
                            print("\n>>> Standaufnahme wird erstellt...")
                            self.snapshot_pointcloud = self.current_pointcloud
                            self.snapshot_marker_position = self.marker_position.copy()
                            self.snapshot_marker_rotation = self.marker_rotation.copy()
                            self.cached_cropped_pointcloud = None
                            self.state = WorkflowState.SNAPSHOT_READY
                            self._print_menu()

                    elif self.state == WorkflowState.SNAPSHOT_READY:
                        if user_input.lower() == "c":
                            print(">>> Abgebrochen")
                            self.state = WorkflowState.PUBLISH_FULL
                            self._print_menu()
                            continue

                        try:
                            parts = user_input.split()
                            if len(parts) == 6:
                                bounds = tuple(float(p) for p in parts)
                                self.crop_bounds = bounds
                                print("\n>>> Crop-Grenzen gesetzt:")
                                print(f"    X: {bounds[0]:.3f} bis {bounds[1]:.3f} m")
                                print(f"    Y: {bounds[2]:.3f} bis {bounds[3]:.3f} m")
                                print(f"    Z: {bounds[4]:.3f} bis {bounds[5]:.3f} m")
                                self._publish_crop_preview_locked()
                                self.state = WorkflowState.CROP_READY
                                self._print_menu()
                            else:
                                print(f">>> FEHLER: Erwartet 6 Zahlen, erhalten {len(parts)}")
                                print(">>> Format: x_min x_max y_min y_max z_min z_max")
                        except ValueError as e:
                            print(f">>> FEHLER: Keine gueltigen Zahlen - {e}")

                    elif self.state == WorkflowState.CROP_READY:
                        if user_input.lower() == "j":
                            if self.cached_cropped_pointcloud is None:
                                print(">>> FEHLER: Kein Crop im Cache. Bitte Crop neu berechnen.")
                                self.state = WorkflowState.SNAPSHOT_READY
                                self._print_menu()
                                continue

                            default_name = f"crop_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                            print(f"Dateiname fuer Snapshot (ohne Endung, Enter={default_name}):")
                            file_name = input().strip() or default_name
                            saved_path = self._save_cropped_snapshot_locked(file_name)

                            if saved_path is not None:
                                print(f">>> Snapshot gespeichert: {saved_path}")
                            else:
                                print(">>> FEHLER: Snapshot konnte nicht gespeichert werden.")

                            print("\n>>> Crop akzeptiert!")
                            print(">>> Starte kontinuierliche Publikation der gecroppten Cloud...")
                            self.state = WorkflowState.PUBLISH_CROPPED
                            self._print_menu()
                        elif user_input.lower() == "n":
                            print("\n>>> Neuer Crop-Bereich erforderlich...")
                            self.crop_bounds = None
                            self.cached_cropped_pointcloud = None
                            self.state = WorkflowState.SNAPSHOT_READY
                            self._print_menu()
                        else:
                            print(">>> Ungueltige Eingabe. Gib 'j' oder 'n' ein.")

                    elif self.state == WorkflowState.PUBLISH_CROPPED:
                        if user_input.lower() == "q":
                            print("\n>>> Programm wird beendet...")
                            self.state = WorkflowState.DONE
                            rclpy.shutdown()
                            return

            except EOFError:
                self.get_logger().info("EOF erreicht, beende Input Loop")
                break
            except Exception as e:
                self.get_logger().error(f"Input Error: {e}")
                break

    def _save_cropped_snapshot_locked(self, file_name):
        """Speichert die gecroppte Cloud als Snapshot (Lock muss gehalten werden)."""
        cloud_msg = self.cached_cropped_pointcloud
        if cloud_msg is None:
            return None

        if not file_name.endswith(".pcd"):
            file_name = f"{file_name}.pcd"

        pcd_path = os.path.join(self.save_dir, file_name)
        xyz, rgb = self._pointcloud2_to_xyz_rgb(cloud_msg)
        if xyz is None or len(xyz) == 0:
            self.get_logger().warn("Keine gueltigen Punkte zum Speichern vorhanden.")
            return None

        try:
            import open3d as o3d

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            if rgb is not None and len(rgb) == len(xyz):
                pcd.colors = o3d.utility.Vector3dVector(rgb)

            ok = o3d.io.write_point_cloud(pcd_path, pcd)
            if not ok:
                raise RuntimeError("Open3D write_point_cloud lieferte False")
            return pcd_path

        except Exception as e:
            self.get_logger().warn(f"Open3D Fehler: {e}. Fallback auf NPY.")
            npy_path = pcd_path.replace(".pcd", ".npy")
            np.save(npy_path, xyz)
            return npy_path

    def _pointcloud2_to_xyz_rgb(self, cloud_msg):
        """Konvertiert PointCloud2 zu XYZ und optional RGB (0..1)."""
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
        """Dekodiert ROS-PointField rgb(float32 packed) zu [r,g,b] in 0..1."""
        rgb_int = struct.unpack("I", struct.pack("f", float(rgb_float)))[0]
        r = ((rgb_int >> 16) & 0xFF) / 255.0
        g = ((rgb_int >> 8) & 0xFF) / 255.0
        b = (rgb_int & 0xFF) / 255.0
        return [r, g, b]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPointCloudCropperSnapshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
