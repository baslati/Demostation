#!/usr/bin/env python3
"""
Bridge between UR3 world frame and camera frame using a shared ArUco reference.

Assumption:
- `aruco_table_ref_0` is the fixed world reference on the table (URDF/static TF).
- `aruco_0` is the measured marker frame from vision (dynamic TF).
- Both represent the same physical marker pose.

The node publishes:
- world -> camera_color_optical_frame

This enables RViz to place incoming camera point clouds in the world frame.
"""

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quat_mul(q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotate_vec(v: Tuple[float, float, float], q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    qv = (v[0], v[1], v[2], 0.0)
    qr = quat_mul(quat_mul(q, qv), quat_conj(q))
    return (qr[0], qr[1], qr[2])


def compose_transform(
    p_ab: Tuple[float, float, float],
    q_ab: Tuple[float, float, float, float],
    p_bc: Tuple[float, float, float],
    q_bc: Tuple[float, float, float, float],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    p_rot = rotate_vec(p_bc, q_ab)
    p_ac = (p_ab[0] + p_rot[0], p_ab[1] + p_rot[1], p_ab[2] + p_rot[2])
    q_ac = quat_mul(q_ab, q_bc)
    return p_ac, q_ac


class ArucoWorldTfBridge(Node):
    def __init__(self) -> None:
        super().__init__("aruco_world_tf_bridge")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("world_marker_frame", "aruco_table_ref_0")
        self.declare_parameter("measured_marker_frame", "aruco_0")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("publish_rate_hz", 20.0)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.world_marker_frame = str(self.get_parameter("world_marker_frame").value)
        self.measured_marker_frame = str(self.get_parameter("measured_marker_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.period = 1.0 / max(1e-3, rate_hz)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._start_time_sec = self.get_clock().now().nanoseconds * 1e-9

        self.create_timer(self.period, self._on_timer)

        self.get_logger().info(
            "TF bridge aktiv: "
            f"{self.world_frame}->{self.camera_frame} aus "
            f"({self.world_frame}->{self.world_marker_frame}) und "
            f"({self.measured_marker_frame}->{self.camera_frame})"
        )

    def _on_timer(self) -> None:
        try:
            tf_world_marker = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.world_marker_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )

            tf_marker_camera = self.tf_buffer.lookup_transform(
                self.measured_marker_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            # Direkt nach Start kommen tf_static-Frames oft verzögert im Buffer an.
            if now_sec - self._start_time_sec < 5.0:
                self.get_logger().info(
                    f"Warte auf TF-Frames ({self.world_frame}, {self.world_marker_frame}, "
                    f"{self.measured_marker_frame}, {self.camera_frame})"
                )
            else:
                self.get_logger().warn(f"TF lookup fehlgeschlagen: {exc}", throttle_duration_sec=2.0)
            return

        p_wm = (
            tf_world_marker.transform.translation.x,
            tf_world_marker.transform.translation.y,
            tf_world_marker.transform.translation.z,
        )
        q_wm = (
            tf_world_marker.transform.rotation.x,
            tf_world_marker.transform.rotation.y,
            tf_world_marker.transform.rotation.z,
            tf_world_marker.transform.rotation.w,
        )

        p_mc = (
            tf_marker_camera.transform.translation.x,
            tf_marker_camera.transform.translation.y,
            tf_marker_camera.transform.translation.z,
        )
        q_mc = (
            tf_marker_camera.transform.rotation.x,
            tf_marker_camera.transform.rotation.y,
            tf_marker_camera.transform.rotation.z,
            tf_marker_camera.transform.rotation.w,
        )

        p_wc, q_wc = compose_transform(p_wm, q_wm, p_mc, q_mc)

        norm = math.sqrt(q_wc[0] ** 2 + q_wc[1] ** 2 + q_wc[2] ** 2 + q_wc[3] ** 2)
        if norm > 1e-9:
            q_wc = (q_wc[0] / norm, q_wc[1] / norm, q_wc[2] / norm, q_wc[3] / norm)

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.world_frame
        out.child_frame_id = self.camera_frame
        out.transform.translation.x = p_wc[0]
        out.transform.translation.y = p_wc[1]
        out.transform.translation.z = p_wc[2]
        out.transform.rotation.x = q_wc[0]
        out.transform.rotation.y = q_wc[1]
        out.transform.rotation.z = q_wc[2]
        out.transform.rotation.w = q_wc[3]

        self.tf_broadcaster.sendTransform(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoWorldTfBridge()
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
