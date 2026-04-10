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
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningSceneComponents,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory


TARGET_TOPIC = "/tool_target_pose"

SERVICE_TIMEOUT_SEC = 8.0
ACTION_TIMEOUT_SEC = 30.0
IK_SERVICE_TIMEOUT_SEC = 20.0
JOINT_STATE_WAIT_SEC = 10.0
STARTUP_MOVE_HOME = True
PLAN_SERVICE_TIMEOUT_SEC = 25.0

# 5 cm ueber Ziel
HOVER_Z_OFFSET_M = 0.02
HOLD_SECONDS = 2.0

# Greifer zeigt standardmaessig nach unten wie in der funktionierenden
# cube_manipulator_gripper_ros.py (Roll = pi, Pitch = 0).
GRIPPER_FIXED_ROLL = math.pi
GRIPPER_FIXED_PITCH = 0.0
TOOL_YAW_OFFSET = 0.0

# Die D405-Integration liefert Tischkoordinaten relativ zum Tisch-Nullpunkt.
# Dieser Nullpunkt liegt in base_link bei (-0.125, +0.125, 0.0) [m]. auf 0.15 wegen ARUCO VERÄNDeRT VON MIR
TARGET_POSE_IS_TABLE_COORDS = True
TABLE_ORIGIN_IN_BASE_X_M = -0.15
TABLE_ORIGIN_IN_BASE_Y_M = 0.15
TABLE_ORIGIN_IN_BASE_Z_M = 0.0
# Achsrichtung Tisch -> base_link.
# X laut Anforderung mit Minus: x_base = origin_x - x_table.
# Y standardmaessig mit Plus:     y_base = origin_y + y_table.
TABLE_TO_BASE_X_SIGN = -1.0
TABLE_TO_BASE_Y_SIGN = -1.0
TABLE_TO_BASE_Z_SIGN = 1.0

# Home-Fahrt nach dem Greifen optional; bei Problemen nicht den Gesamtablauf
# als fehlgeschlagen markieren.
RETURN_HOME_AFTER_GRIP = True

# Home Pose in Tischkoordinaten (gleiches Format wie /tool_target_pose).
# Vorgabe: 19 6 20 (cm) -> 0.19 0.06 0.20 (m)
HOME_TABLE_X_M = 0.17
HOME_TABLE_Y_M = -0.04
HOME_TABLE_Z_M = 0.14

# Home wird mit demselben Orientierungsschema gefahren wie die Zielpose:
# fixer Roll/Pitch (Greifer nach unten) + Yaw um 180 Grad gedreht.
HOME_YAW_RAD = math.pi

# Mapping von ArUco-Frame nach base_link
# Aruco Marker oben links (Tisch-Ursprung)
ARUCO_IN_BASE_X_M = 0.15
ARUCO_IN_BASE_Y_M = 0.15
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


def normalize_frame_id(frame_id: str) -> str:
    """Normalisiert Header-Frames (z. B. '/aruco_0' -> 'aruco_0')."""
    return frame_id.strip().lstrip("/")


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

        self.cli_ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cli_plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cli_fk = self.create_client(GetPositionFK, "/compute_fk")
        self.cli_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.exec_ac = ActionClient(self, FollowJointTrajectory, self.action_name)

        for cli in [self.cli_ik, self.cli_plan, self.cli_fk, self.cli_scene]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("MoveIt Service nicht verfuegbar")
        self.get_logger().info("[INIT] ✓ Alle MoveIt Services gefunden")
        
        if not self.exec_ac.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Trajectory Action Server nicht verfuegbar")
        self.get_logger().info("[INIT] ✓ Trajectory Action Server gefunden")

        self.busy_lock = threading.Lock()
        self.busy = False
        self.pose_received_count = 0
        self.count_lock = threading.Lock()
        self.js_lock = threading.Lock()
        self.latest_joint_state = None
        self.startup_done = False

        # QoS-Profil fuer robuste inter-container Kommunikation
        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.get_logger().info(f"[SUBSCRIPTION] Abonniere Topic: {TARGET_TOPIC} (PoseStamped, QoS=BEST_EFFORT)")
        self.create_subscription(PoseStamped, TARGET_TOPIC, self._on_target_pose, qos_pose)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        
        self.create_timer(2.0, self._subscription_health_check)

        # Start-Check nach dem eigentlichen Spin starten (kein Service-Deadlock in __init__).
        self.create_timer(0.5, self._startup_home_once)
        
        self.get_logger().info("═══════════════════════════════════════════════════════")
        self.get_logger().info("  ✓ NODE BEREIT - WARTE AUF ZIELPOSE")
        self.get_logger().info("═══════════════════════════════════════════════════════")
        self.get_logger().info("Erwartete Topic: /tool_target_pose")
        self.get_logger().info("Erwarteter Frame: aruco_0 oder base_link")
        self.get_logger().info("Warte auf erste Nachricht...")
        self.get_logger().info("Hinweis: Mit 'ros2 topic echo /tool_target_pose' in einem anderen Terminal testen!")

    def _wait_future_result(self, fut, label: str, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.01)
        if not fut.done():
            raise RuntimeError(f"Timeout bei {label} nach {timeout_sec:.1f}s")
        exc = fut.exception()
        if exc is not None:
            raise RuntimeError(f"{label} Exception: {exc}")
        return fut.result()

    def _on_joint_state(self, msg: JointState) -> None:
        with self.js_lock:
            self.latest_joint_state = msg

    def _wait_for_joint_state(self, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            with self.js_lock:
                if self.latest_joint_state is not None and self.latest_joint_state.name:
                    return True
            time.sleep(0.05)
        return False

    def _startup_home_once(self) -> None:
        if self.startup_done:
            return
        self.startup_done = True

        if not STARTUP_MOVE_HOME:
            return

        def worker() -> None:
            self.get_logger().info("[STARTUP] Warte auf /joint_states fuer Initialfahrt...")
            if not self._wait_for_joint_state(JOINT_STATE_WAIT_SEC):
                self.get_logger().warn(
                    f"[STARTUP] Keine /joint_states innerhalb von {JOINT_STATE_WAIT_SEC:.1f}s - "
                    "Initialfahrt zur Home-Pose uebersprungen"
                )
                return
            try:
                self.get_logger().info("[STARTUP] Fahre zur Home-/Scan-Pose zur Funktionspruefung...")
                self._move_home()
                self.get_logger().info("[STARTUP] ✓ Initialfahrt erfolgreich")
            except Exception as exc:
                self.get_logger().error(f"[STARTUP] Initialfahrt fehlgeschlagen: {exc}")

        threading.Thread(target=worker, daemon=True).start()

    def _plan_pose_goal_direct(self, x: float, y: float, z: float, q: Quaternion) -> JointTrajectory:
        constraints = Constraints()

        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base
        pos_c.link_name = self.ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        pos_c.constraint_region = BoundingVolume()
        pos_c.constraint_region.primitives = [sphere]
        pos_c.constraint_region.primitive_poses = [
            PoseStamped(
                header=PoseStamped().header,
                pose=PoseStamped().pose,
            ).pose
        ]
        pos_c.constraint_region.primitive_poses[0].position.x = float(x)
        pos_c.constraint_region.primitive_poses[0].position.y = float(y)
        pos_c.constraint_region.primitive_poses[0].position.z = float(z)
        pos_c.constraint_region.primitive_poses[0].orientation.w = 1.0
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.base
        ori_c.link_name = self.ee_link
        ori_c.orientation = q
        ori_c.absolute_x_axis_tolerance = 0.2
        ori_c.absolute_y_axis_tolerance = 0.2
        ori_c.absolute_z_axis_tolerance = 0.3
        ori_c.weight = 1.0

        constraints.position_constraints = [pos_c]
        constraints.orientation_constraints = [ori_c]

        mpr = MotionPlanRequest()
        mpr.group_name = self.group
        mpr.goal_constraints = [constraints]
        mpr.start_state = self.get_robot_state()
        mpr.max_velocity_scaling_factor = 0.35
        mpr.max_acceleration_scaling_factor = 0.35
        mpr.allowed_planning_time = 15.0
        mpr.num_planning_attempts = 10

        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = mpr
        fut = self.cli_plan.call_async(plan_req)
        plan_res = self._wait_future_result(fut, "GetMotionPlan(direct-pose)", PLAN_SERVICE_TIMEOUT_SEC)
        if plan_res is None or plan_res.motion_plan_response is None:
            raise RuntimeError("GetMotionPlan(direct-pose) lieferte keine Antwort")

        jt = plan_res.motion_plan_response.trajectory.joint_trajectory
        if not jt.points:
            raise RuntimeError("Keine Trajektorie gefunden (direct-pose)")
        return jt

    def get_robot_state(self) -> RobotState:
        # Bevorzugt den RobotState aus MoveIt wie in der funktionierenden
        # Referenzdatei. Falls das fehlschlaegt, verwenden wir den letzten /joint_states Fallback.
        try:
            req = GetPlanningScene.Request()
            req.components = PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE)
            fut = self.cli_scene.call_async(req)
            res = self._wait_future_result(fut, "GetPlanningScene", SERVICE_TIMEOUT_SEC)
            if res is not None and res.scene is not None:
                return res.scene.robot_state
        except Exception as exc:
            self.get_logger().warn(f"[STATE] GetPlanningScene fehlgeschlagen, nutze /joint_states Fallback: {exc}")

        with self.js_lock:
            js = self.latest_joint_state

        if js is None or not js.name:
            raise RuntimeError("Noch keine /joint_states erhalten")

        rs = RobotState()
        rs.joint_state.name = list(js.name)
        rs.joint_state.position = list(js.position)
        rs.joint_state.velocity = list(js.velocity)
        rs.joint_state.effort = list(js.effort)
        rs.joint_state.header = js.header
        return rs

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
        ik_req.ik_request.timeout = Duration(sec=2, nanosec=0)

        ik_res = None
        last_ik_error = None
        for avoid_collisions in (True, False):
            ik_req.ik_request.avoid_collisions = avoid_collisions
            self.get_logger().info(
                f"[IK] ComputeIK (avoid_collisions={avoid_collisions})..."
            )
            fut = self.cli_ik.call_async(ik_req)
            try:
                candidate = self._wait_future_result(fut, "ComputeIK", IK_SERVICE_TIMEOUT_SEC)
            except Exception as exc:
                last_ik_error = exc
                self.get_logger().warn(f"[IK] Versuch fehlgeschlagen: {exc}")
                continue

            if candidate is None:
                last_ik_error = RuntimeError("ComputeIK lieferte keine Antwort")
                continue

            if candidate.error_code.val == candidate.error_code.SUCCESS:
                ik_res = candidate
                break

            last_ik_error = RuntimeError(f"IK error_code={candidate.error_code.val}")

        if ik_res is None:
            self.get_logger().warn(f"[IK] Kein IK-Ergebnis, nutze direct-pose Fallback: {last_ik_error}")
            return self._plan_pose_goal_direct(x, y, z, q)

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
        mpr.allowed_planning_time = 5.0
        mpr.num_planning_attempts = 3

        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = mpr
        fut = self.cli_plan.call_async(plan_req)
        plan_res = self._wait_future_result(fut, "GetMotionPlan", SERVICE_TIMEOUT_SEC)
        if plan_res is None or plan_res.motion_plan_response is None:
            raise RuntimeError("GetMotionPlan lieferte keine Antwort")

        jt = plan_res.motion_plan_response.trajectory.joint_trajectory
        if not jt.points:
            raise RuntimeError("Keine Trajektorie gefunden")
        return jt

    def execute_trajectory(self, jt: JointTrajectory) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        fut = self.exec_ac.send_goal_async(goal)
        gh = self._wait_future_result(fut, "SendGoal", ACTION_TIMEOUT_SEC)
        if gh is None or not gh.accepted:
            raise RuntimeError("Goal abgelehnt")

        res_fut = gh.get_result_async()
        result = self._wait_future_result(res_fut, "GetResult", ACTION_TIMEOUT_SEC)
        if result is None or getattr(result.result, "error_code", 0) != 0:
            raise RuntimeError("Ausfuehrung fehlgeschlagen")

    def _target_in_base(self, msg: PoseStamped) -> Tuple[np.ndarray, Quaternion]:
        p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=np.float64)

        frame = normalize_frame_id(msg.header.frame_id)

        q = msg.pose.orientation
        r_target = quat_to_rotmat(q.x, q.y, q.z, q.w)

        if frame == self.base:
            q_out = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
            return p, q_out

        if frame != "aruco_0":
            raise RuntimeError(f"Unbekannter Eingangsframe: {msg.header.frame_id}")

        if TARGET_POSE_IS_TABLE_COORDS:
            # Tischkoordinaten auf base_link abbilden via bekanntem Tisch-Nullpunkt.
            p_base = np.array([
                TABLE_ORIGIN_IN_BASE_X_M + (TABLE_TO_BASE_X_SIGN * p[0]),
                TABLE_ORIGIN_IN_BASE_Y_M + (TABLE_TO_BASE_Y_SIGN * p[1]),
                TABLE_ORIGIN_IN_BASE_Z_M + (TABLE_TO_BASE_Z_SIGN * p[2]),
            ], dtype=np.float64)
            q_out = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
            self.get_logger().info(
                "[TRANSFORM] aruco_0 als Tischkoordinaten interpretiert "
                f"(Offset base: {TABLE_ORIGIN_IN_BASE_X_M:+.3f}, {TABLE_ORIGIN_IN_BASE_Y_M:+.3f}, {TABLE_ORIGIN_IN_BASE_Z_M:+.3f}; "
                f"Signs x/y/z: {TABLE_TO_BASE_X_SIGN:+.0f}/{TABLE_TO_BASE_Y_SIGN:+.0f}/{TABLE_TO_BASE_Z_SIGN:+.0f})"
            )
            return p_base, q_out

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
        # Gleiches Schema wie bei der Zielpose: fixer Roll/Pitch + Yaw.
        q_home = rpy_to_quat(
            GRIPPER_FIXED_ROLL,
            GRIPPER_FIXED_PITCH,
            HOME_YAW_RAD + TOOL_YAW_OFFSET,
        )
        home_x = TABLE_ORIGIN_IN_BASE_X_M + (TABLE_TO_BASE_X_SIGN * HOME_TABLE_X_M)
        home_y = TABLE_ORIGIN_IN_BASE_Y_M + (TABLE_TO_BASE_Y_SIGN * HOME_TABLE_Y_M)
        home_z = TABLE_ORIGIN_IN_BASE_Z_M + (TABLE_TO_BASE_Z_SIGN * HOME_TABLE_Z_M)
        self.get_logger().info(
            "[HOME] Planung mit Ziel-Schema: "
            f"roll={GRIPPER_FIXED_ROLL:.3f}, pitch={GRIPPER_FIXED_PITCH:.3f}, yaw={HOME_YAW_RAD + TOOL_YAW_OFFSET:.3f}"
        )
        self.get_logger().info(
            "[HOME] Tischkoordinaten -> base_link: "
            f"table=({HOME_TABLE_X_M:.3f}, {HOME_TABLE_Y_M:.3f}, {HOME_TABLE_Z_M:.3f}) -> "
            f"base=({home_x:.3f}, {home_y:.3f}, {home_z:.3f})"
        )
        jt_home = self.plan_to_pose_quat(home_x, home_y, home_z, q_home)
        self.execute_trajectory(jt_home)
        self.get_logger().info("Zur Home-Pose gefahren")

    def _subscription_health_check(self) -> None:
        """Periodischer Check ob Nachrichten ankommen (alle 2s)."""
        with self.count_lock:
            count = self.pose_received_count
        if count == 0:
            self.get_logger().warn(
                "[DIAGNOSE] Noch keine Nachrichten empfangen. "
                "Pruefen: ROS_DOMAIN_ID, 'ros2 topic echo /tool_target_pose'"
            )
        else:
            self.get_logger().debug(f"[DIAGNOSE] {count} Pose-Nachrichten bisher empfangen")

    def _process_target_pose(self, msg: PoseStamped) -> None:
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
            
            if RETURN_HOME_AFTER_GRIP:
                self.get_logger().info("[HOME] Fahre zur Home-Position zurueck...")
                try:
                    self._move_home()
                    self.get_logger().info("[HOME] ✓ Home-Position erreicht")
                except Exception as exc:
                    self.get_logger().warn(f"[HOME] Home-Fahrt fehlgeschlagen (Ablauf bleibt erfolgreich): {exc}")
            else:
                self.get_logger().info("[HOME] Rueckfahrt deaktiviert (RETURN_HOME_AFTER_GRIP=False)")
            
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

    def _on_target_pose(self, msg: PoseStamped) -> None:
        with self.count_lock:
            self.pose_received_count += 1

        norm_frame = normalize_frame_id(msg.header.frame_id)
        if norm_frame != msg.header.frame_id:
            self.get_logger().warn(
                f"[FRAME] Normalisiere frame_id '{msg.header.frame_id}' -> '{norm_frame}'"
            )
            msg.header.frame_id = norm_frame
        
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

        worker_msg = PoseStamped()
        worker_msg.header = msg.header
        worker_msg.pose = msg.pose
        threading.Thread(target=self._process_target_pose, args=(worker_msg,), daemon=True).start()


def set_tool_do(node: Node, pin: int, state: float, timeout: float = 5.0) -> bool:
    from ur_msgs.srv import SetIO

    cli = getattr(node, "_set_io_client", None)
    if cli is None:
        cli = node.create_client(SetIO, "/io_and_status_controller/set_io")
        setattr(node, "_set_io_client", cli)

    if not cli.wait_for_service(timeout_sec=timeout):
        node.get_logger().error("Service /io_and_status_controller/set_io nicht verfuegbar")
        return False

    req = SetIO.Request()
    req.fun = 1
    req.pin = int(pin)
    req.state = float(state)

    fut = cli.call_async(req)

    # Kein spin_until_future_complete hier: die Node laeuft bereits im Executor,
    # und dieser Aufruf kommt aus einem Worker-Thread.
    waiter = getattr(node, "_wait_future_result", None)
    if callable(waiter):
        try:
            res = waiter(fut, "SetIO", timeout)
        except Exception as exc:
            node.get_logger().error(f"SetIO fehlgeschlagen: {exc}")
            return False
    else:
        deadline = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.01)
        if not fut.done():
            node.get_logger().error(f"SetIO Timeout nach {timeout:.1f}s")
            return False
        exc = fut.exception()
        if exc is not None:
            node.get_logger().error(f"SetIO Exception: {exc}")
            return False
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
