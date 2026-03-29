#!/usr/bin/env python3
"""
ArUco Marker basiertes Point Cloud Cropping Tool
Interaktiver 5-Schritt Workflow zur Erfassung und Cropping von Point Clouds
"""
import numpy as np
import cv2
import threading
from enum import Enum
import copy
import struct
import io

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class WorkflowState(Enum):
    """Zustand der Point Cloud Cropping Workflow"""
    PUBLISH_FULL = 1          # Schritt 1: Ganze Pointcloud publishen
    SNAPSHOT_READY = 2        # Schritt 2: Snapshot gespeichert, auf Input warten
    INPUT_CROP_PARAMS = 3     # Schritt 3: XYZ Bereich eingeben
    CROP_READY = 4            # Schritt 4: Crop anzeigen
    CONFIRM_CROP = 5          # Schritt 5: Bestätigung ob Crop passt
    PUBLISH_CROPPED = 6       # Schritt 6: Gecroppte Pointcloud publishen
    DONE = 7                  # Fertig


class ArucoPointCloudCropper(Node):
    def __init__(self):
        super().__init__("aruco_pointcloud_cropper")

        # Parameter Definition
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("pointcloud_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("frame_prefix", "aruco_")
        self.declare_parameter("marker_id_for_crop", 0)

        # Hole Parameter
        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        dict_name = self.get_parameter("dictionary").value
        self.frame_prefix = self.get_parameter("frame_prefix").value
        self.target_marker_id = self.get_parameter("marker_id_for_crop").value

        # Dictionary Validierung
        if dict_name not in DICT_MAP:
            self.get_logger().warn(f"Unbekanntes Dictionary {dict_name}, nutze DICT_4X4_50")
            dict_name = "DICT_4X4_50"

        # ROS Setup
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[dict_name])
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        # Kamera Parameter
        self.k = None
        self.dist = None
        self.camera_frame = "camera_color_optical_frame"

        # Subscriptions
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )

        # Publishers
        self.pub_cropped_cloud = self.create_publisher(PointCloud2, "/cloud_cropped", 10)
        self.pub_full_cloud = self.create_publisher(PointCloud2, "/cloud_full", 10)
        self.pub_axes_marker = self.create_publisher(MarkerArray, "/snapshot_axes", 10)

        # Workflow State
        self.state = WorkflowState.PUBLISH_FULL
        self.current_pointcloud = None
        self.snapshot_pointcloud = None
        self.marker_position = None
        self.marker_rotation = None
        self.snapshot_marker_position = None
        self.snapshot_marker_rotation = None
        self.cached_cropped_pointcloud = None
        self.crop_bounds = None  # (x_min, x_max, y_min, y_max, z_min, z_max) im Marker Frame
        self.lock = threading.Lock()
        self._got_first_cloud = False
        self.publish_timer = self.create_timer(1.0, self._publish_outputs_timer)

        # Starte Input Thread
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info(
            f"ArUco Point Cloud Cropper gestartet.\n"
            f"  Image: {image_topic}\n"
            f"  PointCloud: {self.pointcloud_topic}\n"
            f"  Marker ID: {self.target_marker_id} ({self.frame_prefix}{self.target_marker_id})"
        )
        self._print_menu()

    def _print_menu(self):
        """Drucke das aktuelle Menü basierend auf State"""
        print("\n" + "="*70)
        if self.state == WorkflowState.PUBLISH_FULL:
            print("SCHRITT 1: Veröffentliche vollständige Point Cloud")
            print("-" * 70)
            print("Die gesamte Point Cloud wird live auf /cloud_full publiziert")
            print("Wenn die Standaufnahme bereit ist, drücke ENTER...")
        
        elif self.state == WorkflowState.SNAPSHOT_READY:
            print("SCHRITT 2: Standaufnahme erstellt")
            print("-" * 70)
            print("Gib den XYZ Crop-Bereich relativ zum Marker ein:")
            print("Format: x_min x_max y_min y_max z_min z_max")
            print("Einheit: Meter")
            print("Beispiel: -0.15 0.15 -0.10 0.10 0.0 0.30")
            print("RViz-Achsenfarben: +X = Rot, +Y = Gruen, +Z = Blau")
            print("-" * 70)
            print("Eingabe (oder 'c' zum Abbrechen):")
        
        elif self.state == WorkflowState.CROP_READY:
            print("SCHRITT 3: Crop angezeigt")
            print("-" * 70)
            print(f"Crop-Grenzen: {self.crop_bounds}")
            print("Die gecroppte Cloud wird live auf /cloud_cropped angezeigt")
            print("Passt der Crop? (j/n)")
        
        elif self.state == WorkflowState.PUBLISH_CROPPED:
            print("SCHRITT 4: Veröffentliche gecroppte Point Cloud")
            print("-" * 70)
            print(f"Crop-Grenzen: {self.crop_bounds}")
            print("Die gecroppte Cloud wird kontinuierlich auf /cloud_cropped publiziert")
            print("Drücke 'q' um das Programm zu beenden...")
        
        print("="*70 + "\n")

    def _input_loop(self):
        """Thread für Benutzer-Eingaben"""
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
                                print(">>> FEHLER: Noch keine Marker-Pose verfügbar!")
                                continue
                            print("\n>>> Standaufnahme wird erstellt...")
                            self.snapshot_pointcloud = copy.deepcopy(self.current_pointcloud)
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
                                print(f"\n>>> Crop-Grenzen gesetzt:")
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
                            print(f">>> FEHLER: Keine gültigen Zahlen - {e}")

                    elif self.state == WorkflowState.CROP_READY:
                        if user_input.lower() == "j":
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
                            print(">>> Ungültige Eingabe. Gib 'j' oder 'n' ein.")

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

    def _on_camera_info(self, msg: CameraInfo):
        """Kamera Intrinsics Callback"""
        if self.k is not None:
            return
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id or self.camera_frame
        self.get_logger().info(f"Kamera-Info erhalten. Frame: {self.camera_frame}")

    def _on_image(self, msg: Image):
        """Image Callback - Erkenne ArUco Marker"""
        # Nach Snapshot ist die Marker-Pose eingefroren, weitere Bildverarbeitung nicht noetig.
        if self.snapshot_pointcloud is not None:
            return

        if self.k is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids, _ = self.detector.detectMarkers(frame)
        
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.k, self.dist
        )

        # Verarbeite jeden erkannten Marker
        for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f"{self.frame_prefix}{int(marker_id)}"

            # Translation
            t.transform.translation.x = float(tvec[0][0])
            t.transform.translation.y = float(tvec[0][1])
            t.transform.translation.z = float(tvec[0][2])

            # Rotation (Rodrigues zu Quaternion)
            rmat, _ = cv2.Rodrigues(rvec[0])
            qx, qy, qz, qw = self._quat_from_rotmat(rmat)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)

            # Speichere Position und Rotation des Target-Markers
            if int(marker_id) == self.target_marker_id:
                with self.lock:
                    self.marker_position = tvec[0].copy()
                    self.marker_rotation = rmat.copy()

    def _on_pointcloud(self, msg: PointCloud2):
        """Point Cloud Callback"""
        with self.lock:
            if not self._got_first_cloud:
                self._got_first_cloud = True
                self.get_logger().info("Erste Point Cloud empfangen.")

            # Nur solange noch kein Snapshot erstellt wurde, aktuelle Cloud nachfuehren.
            if self.snapshot_pointcloud is None:
                self.current_pointcloud = msg

    def _publish_outputs_timer(self):
        """Publiziert full/cropped kontinuierlich, unabhaengig von eingehenden Sensor-Callbacks."""
        with self.lock:
            # Vor Snapshot: volle Cloud live durchreichen
            if self.snapshot_pointcloud is None:
                if self.current_pointcloud is not None:
                    self.pub_full_cloud.publish(self.current_pointcloud)
                return

            # Nach Snapshot: Full nur bis inkl. Vorschau senden, in Schritt 4 stoppen.
            if self.state != WorkflowState.PUBLISH_CROPPED:
                self.pub_full_cloud.publish(self.snapshot_pointcloud)

            # Koordinatenkreuz am Snapshot-Marker dauerhaft publizieren.
            axes_marker = self._build_snapshot_axes_markers_locked(self.snapshot_pointcloud.header)
            if axes_marker is not None:
                self.pub_axes_marker.publish(axes_marker)

            # Crop nur noch aus Cache streamen (Berechnung erfolgt bei Eingabe).
            if self.state in [WorkflowState.CROP_READY, WorkflowState.PUBLISH_CROPPED]:
                if self.cached_cropped_pointcloud is not None:
                    self.pub_cropped_cloud.publish(self.cached_cropped_pointcloud)

    def _build_snapshot_axes_markers_locked(self, header):
        """Erstellt drei Marker-Pfeile (+X,+Y,+Z) am Snapshot-Marker."""
        if self.snapshot_marker_position is None or self.snapshot_marker_rotation is None:
            return None

        origin = np.array(self.snapshot_marker_position, dtype=np.float64)
        rmat = np.array(self.snapshot_marker_rotation, dtype=np.float64)
        axis_len = float(max(self.marker_size * 0.8, 0.03))
        shaft_diameter = float(max(self.marker_size * 0.08, 0.003))
        head_diameter = shaft_diameter * 1.8
        head_length = axis_len * 0.2

        marker_array = MarkerArray()

        # Standard-RViz-Farben: X rot, Y gruen, Z blau
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
        """Publiziert sofort eine Crop-Vorschau (Lock muss bereits gehalten werden)."""
        if not self.snapshot_pointcloud or not self.crop_bounds:
            return
        if self.snapshot_marker_position is None or self.snapshot_marker_rotation is None:
            self.get_logger().warn("Keine Snapshot-Marker-Pose verfügbar, Crop-Vorschau nicht möglich.")
            return

        self.cached_cropped_pointcloud = self._crop_pointcloud(self.snapshot_pointcloud)
        if self.cached_cropped_pointcloud is not None:
            self.get_logger().info(
                f"Crop berechnet: {self.cached_cropped_pointcloud.width} Punkte im Bereich."
            )
            self.pub_cropped_cloud.publish(self.cached_cropped_pointcloud)
        else:
            self.get_logger().warn("Crop berechnet: 0 Punkte im Bereich.")

    def _crop_pointcloud(self, cloud_msg: PointCloud2) -> PointCloud2:
        """
        Croppe die Point Cloud basierend auf Grenzen im Marker Frame
        """
        if self.snapshot_marker_position is None or self.snapshot_marker_rotation is None or self.crop_bounds is None:
            return None

        try:
            # Konvertiere Point Cloud zu XYZ + Roh-Point-Daten, damit Farbe erhalten bleibt
            points, point_records = self._pointcloud2_to_xyz_and_records(cloud_msg)
            if points is None or len(points) == 0:
                return None

            x_min, x_max, y_min, y_max, z_min, z_max = self.crop_bounds

            # Transformiere Punkte zum Marker Frame
            # Transformation: P_marker = R_marker^T * (P_camera - t_marker)
            points_relative = points - self.snapshot_marker_position
            points_in_marker_frame = points_relative @ self.snapshot_marker_rotation.T

            # Erstelle Maske für Punkte innerhalb des Bereichs
            mask = (
                (points_in_marker_frame[:, 0] >= x_min) & 
                (points_in_marker_frame[:, 0] <= x_max) &
                (points_in_marker_frame[:, 1] >= y_min) & 
                (points_in_marker_frame[:, 1] <= y_max) &
                (points_in_marker_frame[:, 2] >= z_min) & 
                (points_in_marker_frame[:, 2] <= z_max)
            )

            # Extrahiere gecroppte Punkte
            cropped_points = points[mask]
            cropped_records = [point_records[i] for i in np.where(mask)[0]]

            if len(cropped_points) == 0:
                self.get_logger().warn("Keine Punkte im definierten Bereich!")
                return None

            # Konvertiere zurück zu PointCloud2
            cropped_msg = self._records_to_pointcloud2(cropped_records, cloud_msg)
            return cropped_msg

        except Exception as e:
            self.get_logger().error(f"Cropping-Fehler: {e}")
            return None

    def _pointcloud2_to_xyz_and_records(self, cloud_msg: PointCloud2):
        """
        Konvertiere PointCloud2 zu:
        1) XYZ-Array (N, 3)
        2) Liste der originalen Point-Bytes pro Punkt (für Farberhalt)
        """
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

            # Lese alle Punkte
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
                    
                    # Filtere ungültige Punkte
                    if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                        points.append([x, y, z])
                        records.append(record)
                except struct.error:
                    continue

            if len(points) == 0:
                return None, None
            
            return np.array(points, dtype=np.float32), records

        except Exception as e:
            self.get_logger().error(f"Conversion Error: {e}")
            return None, None

    def _records_to_pointcloud2(self, records, template_msg: PointCloud2) -> PointCloud2:
        """
        Baut PointCloud2 aus originalen Point-Records auf.
        Dadurch bleiben Felder wie rgb/intensity erhalten.
        """
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

        except Exception as e:
            self.get_logger().error(f"PointCloud2 Creation Error: {e}")
            return None

    @staticmethod
    def _quat_from_rotmat(r):
        """
        Konvertiere 3x3 Rotationsmatrix zu Quaternion (x, y, z, w)
        """
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


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPointCloudCropper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
