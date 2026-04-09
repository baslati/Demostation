#!/usr/bin/env python3
"""
D405 Single-Shot Capture Node

Ablauf:
1) Wartet auf Enter im Terminal
2) Liest TFs fuer ArUco und Detektion
3) Publiziert Zielpose relativ zum ArUco-Frame auf /tool_target_pose
4) Publiziert die zuletzt gefundenen TFs weiter (Latch)
"""

import threading
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf2_geometry_msgs import do_transform_pose


TF_TOPIC = "/tf"
TARGET_TOPIC = "/tool_target_pose"
STATUS_TOPIC = "/tool_detection_status"

MARKER_FRAME = "aruco_0"
DETECTED_FRAME = "detected_cropv1_center"

CAPTURE_TIMEOUT_SEC = 6.0


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


class D405CaptureToPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("d405_capture_to_pose_node")

        self.pose_pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        # TF2 Buffer + Listener fuer Frame-Transformationen (verwaltet TFMessage subscription automatisch)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Nach erfolgreichem Scan werden die letzten TFs weiter publiziert,
        # damit ArUco- und Zangen-Frame auch nach Stop des Matching-Prozesses verfuegbar bleiben.
        self.last_marker_tf = None
        self.last_detected_tf = None
        self.last_tf_lock = threading.Lock()
        self.tf_republish_timer = self.create_timer(0.2, self._republish_last_tfs)
        self.capture_active = False

        self._shutdown = False

        self.get_logger().info("D405 Capture Node gestartet")
        self.get_logger().info("Enter -> Capture | q + Enter -> Beenden")

        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()

    def _input_loop(self) -> None:
        while not self._shutdown and rclpy.ok():
            try:
                user = input("\n[Capture] Enter druecken fuer Aufnahme (q zum Beenden): ").strip().lower()
            except EOFError:
                break
            except KeyboardInterrupt:
                break

            if user == "q":
                self.get_logger().info("Beende auf Benutzerwunsch")
                rclpy.shutdown()
                return

            self._run_single_capture()

    def _run_single_capture(self) -> None:
        self.capture_active = True
        self._publish_status("CAPTURE_START")
        self.get_logger().info("═══ CAPTURE START ═══")

        self.get_logger().info(f"Warte auf Pose (Timeout: {CAPTURE_TIMEOUT_SEC}s)...")
        success = self._wait_and_publish_pose(CAPTURE_TIMEOUT_SEC)
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
            
            # Versuche TFs direkt vom Buffer abzurufen
            try:
                marker_tf = self.tf_buffer.lookup_transform(
                    "camera_color_optical_frame",
                    MARKER_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                if self._stamp_is_older_than(marker_tf.header.stamp, capture_start_stamp):
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue
                if log_once_marker:
                    self.get_logger().info(f"✓ ArUco Marker erkannt: {MARKER_FRAME}")
                    log_once_marker = False
            except Exception:
                if log_once_marker:
                    self.get_logger().debug(f"[{elapsed:.1f}s] Warte auf {MARKER_FRAME}...")
                    log_once_marker = False
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            
            try:
                detected_tf = self.tf_buffer.lookup_transform(
                    "camera_depth_optical_frame",
                    DETECTED_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                if self._stamp_is_older_than(detected_tf.header.stamp, capture_start_stamp):
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue
                if log_once_detected:
                    self.get_logger().info(f"✓ Zange erkannt: {DETECTED_FRAME}")
                    log_once_detected = False
            except Exception:
                if log_once_detected:
                    self.get_logger().debug(f"[{elapsed:.1f}s] {MARKER_FRAME} da, warte auf {DETECTED_FRAME}...")
                    log_once_detected = False
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            
            # Beide TFs vorhanden
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
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            rclpy.spin_once(self, timeout_sec=0.05)
        
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
        """Komponiert Pose aus TransformStamped Objekten (vom tf_buffer.lookup_transform)"""
        marker_parent = marker_ts.header.frame_id
        detected_parent = detected_ts.header.frame_id
        
        # Wenn Frame-Mismatch: detected in marker_parent transformieren
        if marker_parent != detected_parent:
            self.get_logger().debug(f"TF-Transformation: {detected_parent} -> {marker_parent}")
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    # detected_ts in den marker_parent Frame umwandeln
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
                    
                    # Transformiere detected in marker_parent Frame
                    detected_transformed = self.tf_buffer.transform(
                        detected_pose, marker_parent, timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    da = detected_transformed.pose.position
                    dq = detected_transformed.pose.orientation
                    break
                except Exception as exc:
                    if attempt < max_retries - 1:
                        self.get_logger().debug(f"TF-Transformation Versuch {attempt+1}: {exc}")
                        time.sleep(0.1)
                    else:
                        self.get_logger().warn(f"TF-Transformation fehlgeschlagen: {exc}")
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

        # camera->marker und camera->detected sind gegeben.
        # Gesucht ist marker->detected.
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
    node = D405CaptureToPoseNode()
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
