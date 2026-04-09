#!/usr/bin/env python3
"""
UR3 Hover + Grip from Pose

Ablauf:
1) Wartet auf Pose auf /tool_target_pose (Frame: aruco_0)
2) Transformiert Pose in base_link
3) Faehert 5 cm ueber Ziel
4) Greifer zu -> 2s halten -> Greifer auf
5) Faehert zur Home-Pose zurueck
"""

import math
import os
import threading
import time
from typing import Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningSceneComponents
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


TARGET_TOPIC = "/tool_target_pose"

# 5 cm ueber Ziel
HOVER_Z_OFFSET_M = 0.05
HOLD_SECONDS = 2.0

# Greifer soll nur in Z-Richtung drehen (Yaw),
# Roll/Pitch bleiben fest, damit er parallel zur Tischoberflaeche bleibt.
GRIPPER_FIXED_ROLL = 0.0
GRIPPER_FIXED_PITCH = 0.0
TOOL_YAW_OFFSET = 0.0

# Benutzerwerte (interpretiert als Home TCP Pose in base_link)
HOME_X_M = 0.320
HOME_Y_M = -0.192
HOME_Z_M = -0.089
HOME_RX = 2.9
HOME_RY = -1.195
HOME_RZ = 0.0

# Mapping von ArUco-Frame nach base_link
# Aruco Marker oben links (Tisch-Ursprung)
ARUCO_IN_BASE_X_M = 0.125
ARUCO_IN_BASE_Y_M = 0.125
ARUCO_IN_BASE_Z_M = 0.0
ARUCO_IN_BASE_RX = 0.0
ARUCO_IN_BASE_RY = 0.0
ARUCO_IN_BASE_RZ = 0.0


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


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


def rpy_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def yaw_from_quat(q: Quaternion) -> float:
    """Extrahiert die planare Yaw-Richtung aus einer Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class UR3HoverGripFromPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("ur3_hover_grip_from_pose_node")
        
        self.get_logger().info("═══════════════════════════════════════════════════════")
        self.get_logger().info("  UR3 HOVER + GRIP FROM POSE NODE STARTEN")
        self.get_logger().info("═══════════════════════════════════════════════════════")

        self.group = "ur_manipulator"
        self.ee_link = "tcp"
        self.base = "base_link"
        self.action_name = "/scaled_joint_trajectory_controller/follow_joint_trajectory"

        self.get_logger().info(f"[INIT] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'nicht gesetzt')}")
        self.get_logger().info(f"[INIT] Suche MoveIt Services und Trajectory Action Server...")

        self.cli_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.cli_ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cli_plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cli_fk = self.create_client(GetPositionFK, "/compute_fk")
        self.exec_ac = ActionClient(self, FollowJointTrajectory, self.action_name)

        for cli in [self.cli_scene, self.cli_ik, self.cli_plan, self.cli_fk]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("MoveIt Service nicht verfuegbar")
        self.get_logger().info("[INIT] ✓ Alle MoveIt Services gefunden")
        
        if not self.exec_ac.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Trajectory Action Server nicht verfuegbar")
        self.get_logger().info("[INIT] ✓ Trajectory Action Server gefunden")

        self.busy_lock = threading.Lock()
        self.busy = False

        self.get_logger().info(f"[SUBSCRIPTION] Abonniere Topic: {TARGET_TOPIC} (PoseStamped, QoS=10)")
        self.create_subscription(PoseStamped, TARGET_TOPIC, self._on_target_pose, 10)
        
        self.get_logger().info("═══════════════════════════════════════════════════════")
        self.get_logger().info("  ✓ NODE BEREIT - WARTE AUF ZIELPOSE")
        self.get_logger().info("═══════════════════════════════════════════════════════")
        self.get_logger().info("Erwartete Topic: /tool_target_pose")
        self.get_logger().info("Erwarteter Frame: aruco_0 oder base_link")
        self.get_logger().info("...")

    def get_robot_state(self):
        req = GetPlanningScene.Request()
        req.components = PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE)
        fut = self.cli_scene.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        return res.scene.robot_state

    def plan_to_pose_quat(self, x: float, y: float, z: float, q: Quaternion) -> JointTrajectory:
        pose = PoseStamped()
        pose.header.frame_id = self.base
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation = q

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = self.group
        ik_req.ik_request.robot_state = self.get_robot_state()
        ik_req.ik_request.pose_stamped = pose
        ik_req.ik_request.ik_link_name = self.ee_link
        ik_req.ik_request.avoid_collisions = True

        fut = self.cli_ik.call_async(ik_req)
        rclpy.spin_until_future_complete(self, fut)
        ik_res = fut.result()
        if ik_res.error_code.val != ik_res.error_code.SUCCESS:
            raise RuntimeError("IK fehlgeschlagen")

        constraints = Constraints()
        for name, pos in zip(ik_res.solution.joint_state.name, ik_res.solution.joint_state.position):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 1e-3
            jc.tolerance_below = 1e-3
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        mpr = MotionPlanRequest()
        mpr.group_name = self.group
        mpr.goal_constraints = [constraints]
        mpr.start_state = self.get_robot_state()
        mpr.max_velocity_scaling_factor = 0.4
        mpr.max_acceleration_scaling_factor = 0.4
        mpr.allowed_planning_time = 2.0

        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = mpr
        fut = self.cli_plan.call_async(plan_req)
        rclpy.spin_until_future_complete(self, fut)
        plan_res = fut.result()

        jt = plan_res.motion_plan_response.trajectory.joint_trajectory
        if not jt.points:
            raise RuntimeError("Keine Trajektorie gefunden")
        return jt

    def execute_trajectory(self, jt: JointTrajectory) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        fut = self.exec_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("Goal abgelehnt")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result()
        if result is None or getattr(result.result, "error_code", 0) != 0:
            raise RuntimeError("Ausfuehrung fehlgeschlagen")

    def _target_in_base(self, msg: PoseStamped) -> Tuple[np.ndarray, Quaternion]:
        p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=np.float64)

        q = msg.pose.orientation
        r_target = quat_to_rotmat(q.x, q.y, q.z, q.w)

        if msg.header.frame_id == self.base:
            q_out = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
            return p, q_out

        if msg.header.frame_id != "aruco_0":
            raise RuntimeError(f"Unbekannter Eingangsframe: {msg.header.frame_id}")

        r_base_aruco = rpy_to_rotmat(ARUCO_IN_BASE_RX, ARUCO_IN_BASE_RY, ARUCO_IN_BASE_RZ)
        p_base_aruco = np.array([ARUCO_IN_BASE_X_M, ARUCO_IN_BASE_Y_M, ARUCO_IN_BASE_Z_M], dtype=np.float64)

        p_base = p_base_aruco + (r_base_aruco @ p)
        r_base_target = r_base_aruco @ r_target
        qx, qy, qz, qw = rotmat_to_quat(r_base_target)

        q_out = Quaternion()
        q_out.x = qx
        q_out.y = qy
        q_out.z = qz
        q_out.w = qw
        return p_base, q_out

    def _move_home(self) -> None:
        q_home = rpy_to_quat(HOME_RX, HOME_RY, HOME_RZ)
        jt_home = self.plan_to_pose_quat(HOME_X_M, HOME_Y_M, HOME_Z_M, q_home)
        self.execute_trajectory(jt_home)
        self.get_logger().info("Zur Home-Pose gefahren")

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.get_logger().info(f"[EMPFANGEN] Zielpose on {TARGET_TOPIC}")
        self.get_logger().info(f"  Frame: {msg.header.frame_id}")
        self.get_logger().info(f"  Zeitstempel: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        self.get_logger().info(f"  Position: x={msg.pose.position.x:.4f}, y={msg.pose.position.y:.4f}, z={msg.pose.position.z:.4f}")
        self.get_logger().info(f"  Quaternion: qx={msg.pose.orientation.x:.4f}, qy={msg.pose.orientation.y:.4f}, qz={msg.pose.orientation.z:.4f}, qw={msg.pose.orientation.w:.4f}")
        
        with self.busy_lock:
            if self.busy:
                self.get_logger().warn("[WARNUNG] Bereits in Bewegung, neue Pose wird ignoriert")
                return
            self.busy = True

        try:
            self.get_logger().info(f"[TRANSFORM] Transformiere Pose von {msg.header.frame_id} -> base_link...")
            p_base, q_base = self._target_in_base(msg)
            self.get_logger().info(f"[TRANSFORM] ✓ Transformation erfolgreich")
            
            hover = p_base.copy()
            hover[2] += HOVER_Z_OFFSET_M
            self.get_logger().info(f"[HOVER] Berechne Hover-Position (+{HOVER_Z_OFFSET_M}m in Z):")
            self.get_logger().info(f"  Ziel in base_link: x={p_base[0]:.4f}, y={p_base[1]:.4f}, z={p_base[2]:.4f}")
            self.get_logger().info(f"  Hover-Position:   x={hover[0]:.4f}, y={hover[1]:.4f}, z={hover[2]:.4f}")

            # Aus Zangenpose nur Yaw uebernehmen; Roll/Pitch bleiben fest.
            target_yaw = yaw_from_quat(q_base) + TOOL_YAW_OFFSET
            q_hover = rpy_to_quat(GRIPPER_FIXED_ROLL, GRIPPER_FIXED_PITCH, target_yaw)
            self.get_logger().info(f"[ORIENT] Greifer-Orientierung = nur Yaw von Tool:")
            self.get_logger().info(f"  Fixe Roll/Pitch (parallel zu Tisch): roll={GRIPPER_FIXED_ROLL:.3f}, pitch={GRIPPER_FIXED_PITCH:.3f}")
            self.get_logger().info(f"  Yaw von Zange (+ Offset {TOOL_YAW_OFFSET}): {target_yaw:.3f} rad")
            self.get_logger().info(f"  Resultat Quaternion: qx={q_hover.x:.4f}, qy={q_hover.y:.4f}, qz={q_hover.z:.4f}, qw={q_hover.w:.4f}")

            self.get_logger().info("[PLANNING] Plane Bewegung zur Hover-Position mit MoveIt...")
            jt_hover = self.plan_to_pose_quat(hover[0], hover[1], hover[2], q_hover)
            self.get_logger().info(f"[PLANNING] ✓ Trajektorie geplant ({len(jt_hover.points)} Punkte)")
            
            self.get_logger().info("[EXECUTION] Fuehre Hover-Trajektorie aus...")
            self.execute_trajectory(jt_hover)
            self.get_logger().info("[EXECUTION] ✓ Hover-Position erreicht")

            self.get_logger().info("[GRIPPER] Greifer wird GESCHLOSSEN...")
            gripper(self, close=True, pulse=False)
            self.get_logger().info(f"[GRIPPER] ✓ Greifer zu - halte {HOLD_SECONDS:.1f}s...")

            time.sleep(HOLD_SECONDS)
            
            self.get_logger().info("[GRIPPER] Greifer wird GEOEFFNET...")
            gripper(self, close=False, pulse=False)
            self.get_logger().info("[GRIPPER] ✓ Greifer offen")
            
            self.get_logger().info("[HOME] Fahre zur Home-Position zurueck...")
            self._move_home()
            self.get_logger().info("[HOME] ✓ Home-Position erreicht")
            
            self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self.get_logger().info("[OK] ABLAUF KOMPLETT ERFOLGREICH - BEREIT FUER NAECHSTE POSE")
            self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

        except Exception as exc:
            self.get_logger().error(f"[ERROR] Ablauf fehlgeschlagen: {exc}", throttle_duration_sec=1)
            import traceback
            self.get_logger().error(f"[ERROR] Traceback:\n{traceback.format_exc()}")
        finally:
            with self.busy_lock:
                self.busy = False


def set_tool_do(node: Node, pin: int, state: float, timeout: float = 5.0) -> bool:
    from ur_msgs.srv import SetIO

    cli = node.create_client(SetIO, "/io_and_status_controller/set_io")
    if not cli.wait_for_service(timeout_sec=timeout):
        node.get_logger().error("Service /io_and_status_controller/set_io nicht verfuegbar")
        return False

    req = SetIO.Request()
    req.fun = 1
    req.pin = int(pin)
    req.state = float(state)

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    res = fut.result()
    return bool(getattr(res, "success", True)) if res is not None else False


def gripper(node: Node, close: bool = True, pin_close: int = 16, pin_open: int = 17,
            pulse_time: float = 1.0, pulse: bool = True, active_high: bool = True) -> None:
    pin = pin_close if close else pin_open
    on = 1.0 if active_high else 0.0
    off = 0.0 if active_high else 1.0

    if not set_tool_do(node, pin, on):
        raise RuntimeError("Tool DO konnte nicht eingeschaltet werden")

    if pulse:
        time.sleep(max(0.0, pulse_time))
        if not set_tool_do(node, pin, off):
            raise RuntimeError("Tool DO konnte nicht ausgeschaltet werden")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UR3HoverGripFromPoseNode()
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
