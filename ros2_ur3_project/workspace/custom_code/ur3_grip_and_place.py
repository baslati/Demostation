#!/usr/bin/env python3
"""
UR3 Top-Down Grip and Template-Based Place

Ablauf:
1) Wartet auf Pose auf /tool_target_pose (Frame: aruco_0|<template_id>)
2) Transformiert Pose in base_link
3) Faehrt zuerst 2 cm ueber die Greifpose (Hover)
4) Faellt von oben vertikal auf die Greifpose ab
5) Faehrt 4 cm in Marker-Y-Richtung (XY-Ebene)
6) Greifer schliessen, Werkzeug aufnehmen
7) Faehrt zu templatespezifischem Ablagepunkt
8) Legt ab, zieht in Ablagerichtung zurueck (template-spezifische Distanz)
9) Zieht vertikal nach oben (template-spezifische Distanz)
10) Optional zur Home-Pose zurueck
"""

import math
import os
import threading
import time
from typing import Dict, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory


TARGET_TOPIC = "/tool_target_pose"

DEFAULT_TEMPLATE_ID = "breitv1_clean_direction"
TEMPLATE_GRASP_OFFSETS: Dict[str, Dict[str, Tuple[float, float, float, float]]] = {
    "cropv1_clean_direction": {
        "translation_xyz_m": (-0.001352, +0.087173, -0.014160),
        "translation_sign_xyz": (+1.0, -1.0, +1.0),
        "rotation_quat_xyzw": (+0.998850, -0.035604, -0.006631, +0.031408),
    },
    "kurzv3_clean_direction": {
        "translation_xyz_m": (0.005, +0.08, -0.011746),
        "translation_sign_xyz": (+1.0, -1.0, +1.0),
        "rotation_quat_xyzw": (+0.999909, +0.001677, -0.011609, +0.006670),
    },
    "breitv1_clean_direction": {
        "translation_xyz_m": (0.0, +0.085391, -0.008295),
        "translation_sign_xyz": (+1.0, -1.0, +1.0),
        "rotation_quat_xyzw": (+0.999901, -0.009287, -0.010443, +0.001564),
    },
    "langv1_clean_direction": {
        "translation_xyz_m": (0.002, +0.108, -0.010080),
        "translation_sign_xyz": (+1.0, -1.0, +1.0),
        "rotation_quat_xyzw": (+0.999927, -0.007771, -0.000057, +0.009240),
    },
}

# Ablagekonfiguration pro Template.
# place_x/y/z: Ablageposition im Marker-/Tischframe (m)
# place_yaw:   Ausrichtung beim Ablegen (rad)
# post_place_forward_m: Rueckwaertsbewegung entlang Ablage-Y nach dem Oeffnen des Greifers
# post_place_retreat_m: Vertikaler Rueckzug danach
#
# Erweiterung fuer spaeter: Eintrag "place_mode": "holder" aktiviert eine
# andere Bewegungssequenz (z.B. Einsetzen in Halterung). Aktuell nur "table".
TEMPLATE_PLACE_CONFIG: Dict[str, Dict] = {
    "breitv1_clean_direction": {
        "place_x": 0.06,#rechts
        "place_y": 0.175,#hoch
        "place_z": 0.0,
        "place_yaw": math.pi*1.1,
        "post_place_forward_m": 0.005,
        "post_place_retreat_m": 0.02,
    },
    "kurzv3_clean_direction": {
        "place_x": 0.20,
        "place_y": -0.06,
        "place_z": 0.0,
        "place_yaw": math.pi,
        "post_place_forward_m": 0.02,
        "post_place_retreat_m": 0.02,
        "place_mode": "drop",
        # Exakte Loslasspose (gemessen mit /tcp_pose_broadcaster/pose, Frame: base)
        "drop_pose_position":    (0.40546671094181586, -0.0394736154984765, 0.18363264835816195),
        "drop_pose_orientation": (-0.7400943638126082, 0.30610878535680625, 0.22317351932409374, 0.555653961069918),  # xyzw
        # Gelenkwinkel der Loslasspose (gemessen mit /joint_states) -> kein IK noetig
        "drop_joint_config": {
            "shoulder_pan_joint":  2.5424118041992188,
            "shoulder_lift_joint": -0.9264412683299561,
            "elbow_joint":          1.1158397833453577,
            "wrist_1_joint":       -0.7221396514560240,
            "wrist_2_joint":       -0.6281092802630823,
            "wrist_3_joint":        4.358260631561279,
        },
    },
    "langv1_clean_direction": {
        #"place_x": 0.20,
        #"place_y": -0.06,
        #"place_z": 0.0,
        #"place_yaw": (math.pi/2+math.pi/8),
        "place_x": 0.16,#rechts
        "place_y": 0.17,#hoch
        "place_z": 0.0,
        "place_yaw": math.pi*1.25,
        "post_place_forward_m": 0.00,
        "post_place_retreat_m": 0.02,
    },
}
DEFAULT_PLACE_CONFIG_KEY = "breitv1_clean_direction"

SERVICE_TIMEOUT_SEC = 8.0
ACTION_TIMEOUT_SEC = 30.0
IK_SERVICE_TIMEOUT_SEC = 20.0
JOINT_STATE_WAIT_SEC = 10.0
STARTUP_MOVE_HOME = True


HOLD_SECONDS = 0.5
OPEN_SECONDS = 0.5

HOVER_ABOVE_GRIP_M = 0.03
AXIS_SHIFT_MARKER_Y_M = 0.05
GRIP_SHIFT_MARKER_Y_M = 0.01

MIN_TARGET_Z_IN_BASE_M = 0.0032
PLANNING_Z_RETRY_STEPS_M = (0.0, 0.008, 0.015)
APPROACH_HOVER_YAW_RETRY_DEG = (0.0, 5.0, -5.0, 10.0, -10.0)
APPROACH_HOVER_PITCH_RETRY_DEG = (0.0, 4.0, -4.0, 8.0, -8.0)

GRIPPER_FIXED_ROLL = math.pi
GRIPPER_FIXED_PITCH = 0.0
TOOL_YAW_OFFSET = 0.0

TABLE_ORIGIN_IN_BASE_X_M = -0.15
TABLE_ORIGIN_IN_BASE_Y_M = 0.15
TABLE_ORIGIN_IN_BASE_Z_M = 0.0
TABLE_TO_BASE_X_SIGN = -1.0
TABLE_TO_BASE_Y_SIGN = -1.0
TABLE_TO_BASE_Z_SIGN = 1.0

RETURN_HOME_AFTER_GRIP = True

HOME_TABLE_X_M = 0.17
HOME_TABLE_Y_M = -0.035
HOME_TABLE_Z_M = 0.15
HOME_ROLL_RAD  =  math.pi
HOME_PITCH_RAD = 0
HOME_YAW_RAD   = math.pi -0.02

POST_GRIP_LIFT_BEFORE_PLACE_M = 0.02

TABLE_CENTER_X_M = 0.0
TABLE_CENTER_Y_M = 0.0
PLACE_MIN_DIST_FROM_CENTER_M = 0.08
PLACE_MIN_DIST_FROM_PICK_M = 0.08

FJT_ERROR_TEXT = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -2: "INVALID_JOINTS",
    -3: "OLD_HEADER_TIMESTAMP",
    -4: "PATH_TOLERANCE_VIOLATED",
    -5: "GOAL_TOLERANCE_VIOLATED",
}


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


def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_frame_id(frame_id: str) -> str:
    return frame_id.strip().lstrip("/")


def split_frame_and_template(frame_id: str) -> Tuple[str, str]:
    norm = normalize_frame_id(frame_id)
    if "|" not in norm:
        return norm, DEFAULT_TEMPLATE_ID
    base_frame, template_id = norm.split("|", 1)
    base_frame = base_frame.strip()
    template_id = template_id.strip() or DEFAULT_TEMPLATE_ID
    return base_frame, template_id


class UR3GripAndPlaceNode(Node):
    def __init__(self) -> None:
        super().__init__("ur3_grip_and_place_node")

        self.get_logger().info("=======================================================")
        self.get_logger().info("  UR3 GRIP AND PLACE BY TEMPLATE NODE START")
        self.get_logger().info("=======================================================")

        self.group = "ur_manipulator"
        self.ee_link = "tcp"
        self.base = "base_link"
        self.action_name = "/scaled_joint_trajectory_controller/follow_joint_trajectory"

        self.get_logger().info(f"[INIT] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'nicht gesetzt')}")
        self.get_logger().info("[INIT] Suche MoveIt Services und Trajectory Action Server...")

        self.cli_ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cli_plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cli_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.exec_ac = ActionClient(self, FollowJointTrajectory, self.action_name)

        for cli in [self.cli_ik, self.cli_plan, self.cli_scene]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("MoveIt Service nicht verfuegbar")
        self.get_logger().info("[INIT] Alle MoveIt Services gefunden")

        if not self.exec_ac.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Trajectory Action Server nicht verfuegbar")
        self.get_logger().info("[INIT] Trajectory Action Server gefunden")

        self.busy_lock = threading.Lock()
        self.busy = False
        self.pose_received_count = 0
        self.count_lock = threading.Lock()
        self.js_lock = threading.Lock()
        self.latest_joint_state = None
        self.startup_done = False

        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.gui_status_pub = self.create_publisher(String, "/gui/robot_status", 10)

        self.get_logger().info(f"[SUBSCRIPTION] Abonniere Topic: {TARGET_TOPIC} (PoseStamped)")
        self.create_subscription(PoseStamped, TARGET_TOPIC, self._on_target_pose, qos_pose)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(Bool, "/gui/home_drive", self._on_home_drive, 10)

        self.create_timer(2.0, self._subscription_health_check)
        self.create_timer(0.5, self._startup_home_once)

        self.get_logger().info("=======================================================")
        self.get_logger().info("  NODE BEREIT - WARTE AUF ZIELPOSE")
        self.get_logger().info("=======================================================")

    def _get_place_config(self, template_id: str) -> Dict:
        cfg = TEMPLATE_PLACE_CONFIG.get(template_id)
        if cfg is None:
            self.get_logger().warn(
                f"[PLACE_CONFIG] Kein Eintrag fuer '{template_id}', nutze '{DEFAULT_PLACE_CONFIG_KEY}'"
            )
            cfg = TEMPLATE_PLACE_CONFIG[DEFAULT_PLACE_CONFIG_KEY]
        return cfg

    def _apply_template_grasp_offset(self, msg: PoseStamped, template_id: str) -> PoseStamped:
        config = TEMPLATE_GRASP_OFFSETS.get(template_id)
        if config is None:
            self.get_logger().warn(
                f"[TEMPLATE] Kein Offset fuer '{template_id}' definiert, nutze '{DEFAULT_TEMPLATE_ID}'"
            )
            config = TEMPLATE_GRASP_OFFSETS[DEFAULT_TEMPLATE_ID]

        t_raw = np.array(config["translation_xyz_m"], dtype=np.float64)
        t_sign = np.array(config.get("translation_sign_xyz", (+1.0, +1.0, +1.0)), dtype=np.float64)
        t_off = t_raw * t_sign
        q_off = config["rotation_quat_xyzw"]

        p_in = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        q_in = msg.pose.orientation
        r_in = quat_to_rotmat(q_in.x, q_in.y, q_in.z, q_in.w)
        r_off = quat_to_rotmat(q_off[0], q_off[1], q_off[2], q_off[3])

        p_grip = p_in + (r_in @ t_off)
        r_grip = r_in @ r_off
        qx, qy, qz, qw = rotmat_to_quat(r_grip)

        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x = float(p_grip[0])
        out.pose.position.y = float(p_grip[1])
        out.pose.position.z = float(p_grip[2])
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        return out

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
                    f"[STARTUP] Keine /joint_states innerhalb von {JOINT_STATE_WAIT_SEC:.1f}s"
                )
                return
            time.sleep(3.0)
            try:
                self.get_logger().info("[STARTUP] Fahre zur Home-Pose...")
                self._move_home()
                self.get_logger().info("[STARTUP] Initialfahrt erfolgreich")
            except Exception as exc:
                self.get_logger().error(f"[STARTUP] Initialfahrt fehlgeschlagen: {exc}")

        threading.Thread(target=worker, daemon=True).start()

    def get_robot_state(self) -> RobotState:
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
            fut = self.cli_ik.call_async(ik_req)
            try:
                candidate = self._wait_future_result(fut, "ComputeIK", IK_SERVICE_TIMEOUT_SEC)
            except Exception as exc:
                last_ik_error = exc
                continue

            if candidate is None:
                last_ik_error = RuntimeError("ComputeIK lieferte keine Antwort")
                continue

            if candidate.error_code.val == candidate.error_code.SUCCESS:
                ik_res = candidate
                break

            last_ik_error = RuntimeError(f"IK error_code={candidate.error_code.val}")

        if ik_res is None:
            raise RuntimeError(f"IK fehlgeschlagen (Ursache: {last_ik_error})")

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

    def plan_to_joint_config(self, joint_config: Dict[str, float]) -> JointTrajectory:
        constraints = Constraints()
        for name, pos in joint_config.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
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
        if result is None:
            raise RuntimeError("Ausfuehrung fehlgeschlagen: kein Ergebnis")

        error_code = int(getattr(result.result, "error_code", 0))
        error_str = str(getattr(result.result, "error_string", "")).strip()
        if error_code != 0:
            code_label = FJT_ERROR_TEXT.get(error_code, "UNBEKANNT")
            details = f"error_code={error_code} ({code_label})"
            if error_str:
                details += f", error_string='{error_str}'"
            self.get_logger().error(f"[EXECUTE] Trajektorie fehlgeschlagen: {details}")
            raise RuntimeError(f"Ausfuehrung fehlgeschlagen: {details}")

    def _target_in_base(self, msg: PoseStamped) -> Tuple[np.ndarray, Quaternion]:
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        frame = normalize_frame_id(msg.header.frame_id)

        q = msg.pose.orientation

        if frame == self.base:
            q_out = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
            return p, q_out

        if frame != "aruco_0":
            raise RuntimeError(f"Unbekannter Eingangsframe: {msg.header.frame_id}")

        p_base = np.array([
            TABLE_ORIGIN_IN_BASE_X_M + (TABLE_TO_BASE_X_SIGN * p[0]),
            TABLE_ORIGIN_IN_BASE_Y_M + (TABLE_TO_BASE_Y_SIGN * p[1]),
            TABLE_ORIGIN_IN_BASE_Z_M + (TABLE_TO_BASE_Z_SIGN * p[2]),
        ], dtype=np.float64)
        q_out = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        return p_base, q_out

    def _table_to_base(self, x_table: float, y_table: float, z_table: float) -> np.ndarray:
        return np.array([
            TABLE_ORIGIN_IN_BASE_X_M + (TABLE_TO_BASE_X_SIGN * x_table),
            TABLE_ORIGIN_IN_BASE_Y_M + (TABLE_TO_BASE_Y_SIGN * y_table),
            TABLE_ORIGIN_IN_BASE_Z_M + (TABLE_TO_BASE_Z_SIGN * z_table),
        ], dtype=np.float64)

    def _get_place_target_base(self, place_cfg: Dict) -> np.ndarray:
        return self._table_to_base(place_cfg["place_x"], place_cfg["place_y"], place_cfg["place_z"])

    def _validate_place_target(self, pick_base: np.ndarray, place_base: np.ndarray) -> None:
        center_base = self._table_to_base(TABLE_CENTER_X_M, TABLE_CENTER_Y_M, 0.0)
        dist_center = float(np.linalg.norm(place_base[:2] - center_base[:2]))
        if dist_center < PLACE_MIN_DIST_FROM_CENTER_M:
            self.get_logger().warn(
                f"[PLACE_TARGET] Nah an Tischmitte: dist={dist_center:.3f}m < {PLACE_MIN_DIST_FROM_CENTER_M:.3f}m (fortsetzen)"
            )

        dist_pick = float(np.linalg.norm(place_base[:2] - pick_base[:2]))
        if dist_pick < PLACE_MIN_DIST_FROM_PICK_M:
            self.get_logger().warn(
                f"[PLACE_TARGET] Nah an Greifstelle: dist={dist_pick:.3f}m < {PLACE_MIN_DIST_FROM_PICK_M:.3f}m (fortsetzen)"
            )

    def _plan_with_retry(self, p: np.ndarray, q_target: Quaternion, label: str) -> Tuple[JointTrajectory, float]:
        planning_errors = []
        for dz in PLANNING_Z_RETRY_STEPS_M:
            z_try = float(p[2] + dz)
            self.get_logger().info(f"[PLANNING] {label}: z={z_try:.4f} (dz={dz:+.3f})")
            try:
                jt = self.plan_to_pose_quat(float(p[0]), float(p[1]), z_try, q_target)
                return jt, z_try
            except Exception as exc:
                planning_errors.append(str(exc))

            if label == "APPROACH_HOVER":
                base_yaw = yaw_from_quat(q_target)
                for yaw_deg in APPROACH_HOVER_YAW_RETRY_DEG[1:]:
                    q_retry = rpy_to_quat(
                        GRIPPER_FIXED_ROLL,
                        GRIPPER_FIXED_PITCH,
                        base_yaw + math.radians(yaw_deg),
                    )
                    try:
                        self.get_logger().info(
                            f"[PLANNING] {label}: retry yaw={yaw_deg:+.1f} deg bei z={z_try:.4f}"
                        )
                        jt = self.plan_to_pose_quat(float(p[0]), float(p[1]), z_try, q_retry)
                        return jt, z_try
                    except Exception as exc:
                        planning_errors.append(str(exc))

                for pitch_deg in APPROACH_HOVER_PITCH_RETRY_DEG[1:]:
                    q_retry = rpy_to_quat(
                        GRIPPER_FIXED_ROLL,
                        GRIPPER_FIXED_PITCH + math.radians(pitch_deg),
                        base_yaw,
                    )
                    try:
                        self.get_logger().info(
                            f"[PLANNING] {label}: retry pitch={pitch_deg:+.1f} deg bei z={z_try:.4f}"
                        )
                        jt = self.plan_to_pose_quat(float(p[0]), float(p[1]), z_try, q_retry)
                        return jt, z_try
                    except Exception as exc:
                        planning_errors.append(str(exc))

        raise RuntimeError(
            f"{label}: keine Trajektorie gefunden; letzte Ursache={planning_errors[-1] if planning_errors else 'unbekannt'}"
        )

    def _move_home(self) -> None:
        q_home = rpy_to_quat(HOME_ROLL_RAD, HOME_PITCH_RAD, HOME_YAW_RAD + TOOL_YAW_OFFSET)
        home = self._table_to_base(HOME_TABLE_X_M, HOME_TABLE_Y_M, HOME_TABLE_Z_M)
        jt_home, _ = self._plan_with_retry(home, q_home, "HOME")
        self.execute_trajectory(jt_home)
        self.get_logger().info("[HOME] Zur Home-Pose gefahren")

    def _on_home_drive(self, msg) -> None:
        if not msg.data:
            return
        with self.busy_lock:
            if self.busy:
                self.get_logger().warn("[HOME-DRIVE] Roboter beschäftigt, Home-Fahrt ignoriert")
                return
            self.busy = True
        def worker():
            try:
                self.get_logger().info("[HOME-DRIVE] Kamerafahrt gestartet")
                self._move_home()
                self.get_logger().info("[HOME-DRIVE] Kamerafahrt abgeschlossen")
                self._publish_gui_status("home_reached")
            except Exception as exc:
                self.get_logger().error(f"[HOME-DRIVE] Fehler: {exc}")
                exc_str = str(exc).lower()
                if "tolerance" in exc_str or "path_tolerance" in exc_str:
                    self._publish_gui_status("tolerance_violation")
                else:
                    self._publish_gui_status("no_path")
            finally:
                with self.busy_lock:
                    self.busy = False
        threading.Thread(target=worker, daemon=True).start()

    def _subscription_health_check(self) -> None:
        with self.count_lock:
            count = self.pose_received_count
        if count == 0:
            self.get_logger().warn(
                "[DIAGNOSE] Noch keine Nachrichten empfangen. "
                "Pruefen: ROS_DOMAIN_ID, 'ros2 topic echo /tool_target_pose'"
            )

    def _process_target_pose(self, msg: PoseStamped) -> None:
        try:
            base_frame, template_id = split_frame_and_template(msg.header.frame_id)
            place_cfg = self._get_place_config(template_id)

            msg_grip = self._apply_template_grasp_offset(msg, template_id)
            msg_grip.header.frame_id = base_frame

            p_base, q_base = self._target_in_base(msg_grip)
            p_grip = p_base.copy()
            p_grip[2] = 0.003
            self.get_logger().info("[GRASP_Z] z-Wert fest auf 0.003 gesetzt")

            target_yaw = yaw_from_quat(q_base) + TOOL_YAW_OFFSET
            q_target = rpy_to_quat(GRIPPER_FIXED_ROLL, GRIPPER_FIXED_PITCH, target_yaw)
            q_place = rpy_to_quat(GRIPPER_FIXED_ROLL, GRIPPER_FIXED_PITCH, place_cfg["place_yaw"] + TOOL_YAW_OFFSET)

            r_base_marker = quat_to_rotmat(q_base.x, q_base.y, q_base.z, q_base.w)
            marker_y_in_base = r_base_marker[:, 1]
            marker_y_xy = np.array([marker_y_in_base[0], marker_y_in_base[1], 0.0], dtype=np.float64)
            norm_xy = float(np.linalg.norm(marker_y_xy))
            if norm_xy < 1e-9:
                raise RuntimeError("Marker +Y kann nicht in XY-Ebene projiziert werden (norm~0)")
            marker_y_xy /= norm_xy

            if abs(GRIP_SHIFT_MARKER_Y_M) > 1e-9:
                grip_delta_xy = marker_y_xy * GRIP_SHIFT_MARKER_Y_M
                p_grip[0] += grip_delta_xy[0]
                p_grip[1] += grip_delta_xy[1]
                self.get_logger().info(
                    f"[GRIP_SHIFT] +Y={GRIP_SHIFT_MARKER_Y_M:.3f}m, delta_xy=({grip_delta_xy[0]:+.4f}, {grip_delta_xy[1]:+.4f})"
                )

            # 1) Hover 2 cm ueber Greifpose
            p_hover = p_grip.copy()
            p_hover[2] += HOVER_ABOVE_GRIP_M
            self.get_logger().info(
                f"[APPROACH_HOVER] Ziel=({p_hover[0]:.4f}, {p_hover[1]:.4f}, {p_hover[2]:.4f})"
            )
            jt_hover, z_hover = self._plan_with_retry(p_hover, q_target, "APPROACH_HOVER")
            p_hover[2] = z_hover
            self.execute_trajectory(jt_hover)

            # 2) Vertikal von oben nach unten
            p_descend = p_grip.copy()
            p_descend[2] = min(p_hover[2], p_descend[2])
            self.get_logger().info(
                f"[DESCEND_VERTICAL] Ziel=({p_descend[0]:.4f}, {p_descend[1]:.4f}, {p_descend[2]:.4f})"
            )
            jt_desc, z_desc = self._plan_with_retry(p_descend, q_target, "DESCEND_VERTICAL")
            p_descend[2] = z_desc
            self.execute_trajectory(jt_desc)

            # 3) 5 cm entlang Marker-Y in XY-Ebene
            delta_xy = marker_y_xy * AXIS_SHIFT_MARKER_Y_M
            p_shift = p_descend.copy()
            p_shift[0] += delta_xy[0]
            p_shift[1] += delta_xy[1]
            self.get_logger().info(
                f"[SHIFT_MARKER_Y_5CM] delta_xy=({delta_xy[0]:+.4f}, {delta_xy[1]:+.4f}), z={p_shift[2]:.4f}"
            )
            jt_shift, z_shift = self._plan_with_retry(p_shift, q_target, "SHIFT_MARKER_Y_5CM")
            p_shift[2] = z_shift
            self.execute_trajectory(jt_shift)

            # 4) Greifen
            self.get_logger().info("[GRIPPER] Schliessen...")
            if not set_tool_do(self, 16, 1.0):
                raise RuntimeError("Greifer CLOSE (Pin 16=1) fehlgeschlagen")
            time.sleep(HOLD_SECONDS)
            self.get_logger().info("[GRIPPER] Geschlossen und gehalten")

            # 5) Nach dem Greifen erst vertikal nach oben
            p_after_grip_up = p_shift.copy()
            p_after_grip_up[2] += POST_GRIP_LIFT_BEFORE_PLACE_M
            self.get_logger().info(
                f"[POST_GRIP_UP] Hebe vertikal um {POST_GRIP_LIFT_BEFORE_PLACE_M:.3f}m auf z={p_after_grip_up[2]:.4f}"
            )
            jt_after_grip_up, z_after_grip_up = self._plan_with_retry(p_after_grip_up, q_target, "POST_GRIP_UP")
            p_after_grip_up[2] = z_after_grip_up
            self.execute_trajectory(jt_after_grip_up)

            # 6) Ablagepunkt (template-spezifisch)
            p_place = self._get_place_target_base(place_cfg)
            if p_place[2] < MIN_TARGET_Z_IN_BASE_M:
                p_place[2] = MIN_TARGET_Z_IN_BASE_M
            self._validate_place_target(p_shift, p_place)

            self.get_logger().info(
                "[PLACE_TARGET] "
                f"table=({place_cfg['place_x']:.4f}, {place_cfg['place_y']:.4f}, {place_cfg['place_z']:.4f}) -> "
                f"base=({p_place[0]:.4f}, {p_place[1]:.4f}, {p_place[2]:.4f}), "
                f"yaw={place_cfg['place_yaw'] + TOOL_YAW_OFFSET:.3f}  [template={template_id}]"
            )

            place_mode = place_cfg.get("place_mode", "table")

            if place_mode == "drop":
                # DROP-Modus: Zange in schraege Halterung fallen lassen
                # Exakte Loslasspose aus Config (gemessen mit TCP-Pose-Tool)
                dp = place_cfg["drop_pose_position"]
                dq_raw = place_cfg["drop_pose_orientation"]  # xyzw
                q_drop = Quaternion(x=dq_raw[0], y=dq_raw[1], z=dq_raw[2], w=dq_raw[3])
                p_drop = np.array([dp[0], dp[1], dp[2]], dtype=np.float64)

                # 5b) Zur Home-Position fahren (XY + Z), dann von dort zur Loslasspose
                self.get_logger().info("[DROP_MOVE_HOME] Fahre zunaechst zur Home-Position")
                self._move_home()

                # 7) Direkt zur gemessenen Loslasspose via Joint-Space (kein IK noetig)
                drop_joints = place_cfg.get("drop_joint_config")
                if drop_joints:
                    self.get_logger().info("[DROP_MOVE] Fahre zur Loslasspose (Joint-Space)")
                    jt_drop = self.plan_to_joint_config(drop_joints)
                else:
                    self.get_logger().info(
                        f"[DROP_MOVE] Fahre zur Loslasspose (IK) "
                        f"({p_drop[0]:.4f}, {p_drop[1]:.4f}, {p_drop[2]:.4f})"
                    )
                    jt_drop, _ = self._plan_with_retry(p_drop, q_drop, "DROP_MOVE")
                self.execute_trajectory(jt_drop)

                # 8) Greifer oeffnen -> Zange faellt ins Fach
                self.get_logger().info("[GRIPPER] Oeffnen (DROP)...")
                if not set_tool_do(self, 16, 0.0):
                    raise RuntimeError("Greifer CLOSE Release (Pin 16=0) fehlgeschlagen")
                gripper(self, close=False, pulse=True, pulse_time=OPEN_SECONDS)
                self.get_logger().info("[GRIPPER] Offen - Zange faellt ins Fach")

                # 9) Direkt zur Home-Pose zurueck (Joint-Space, kein IK noetig)
                self.get_logger().info("[DROP_RETREAT] Fahre direkt zur Home-Pose")
                self._move_home()

            else:
                # TABLE-Modus: normales Ablegen auf dem Tisch
                # 7) In XY-Ebene zur Ablage verfahren und dabei bereits auf Ablageausrichtung drehen
                p_place_plane = p_place.copy()
                p_place_plane[2] = p_after_grip_up[2]
                self.get_logger().info("[PLACE_MOVE_ALIGN] Verfahre in Ebene zur Ablage und richte aus")
                jt_place_plane, z_place_plane = self._plan_with_retry(p_place_plane, q_place, "PLACE_MOVE_ALIGN")
                p_place_plane[2] = z_place_plane
                self.execute_trajectory(jt_place_plane)

                # 8) Direkt vertikal auf Ablagehoehe
                self.get_logger().info("[PLACE_DESCEND] Senke direkt auf Ablagehoehe")
                jt_place_desc, z_place_desc = self._plan_with_retry(p_place, q_place, "PLACE_DESCEND")
                p_place[2] = z_place_desc
                self.execute_trajectory(jt_place_desc)

                # 9) Ablegen
                self.get_logger().info("[GRIPPER] Oeffnen...")
                if not set_tool_do(self, 16, 0.0):
                    raise RuntimeError("Greifer CLOSE Release (Pin 16=0) fehlgeschlagen")
                gripper(self, close=False, pulse=True, pulse_time=OPEN_SECONDS)
                self.get_logger().info("[GRIPPER] Offen")

                # 10) In Ablageausrichtung zurueckfahren (template-spezifische Distanz)
                r_base_place = quat_to_rotmat(q_place.x, q_place.y, q_place.z, q_place.w)
                place_y_in_base = r_base_place[:, 1]
                place_y_xy = np.array([place_y_in_base[0], place_y_in_base[1], 0.0], dtype=np.float64)
                place_norm_xy = float(np.linalg.norm(place_y_xy))
                if place_norm_xy < 1e-9:
                    raise RuntimeError("Ablage +Y kann nicht in XY-Ebene projiziert werden (norm~0)")
                place_y_xy /= place_norm_xy

                post_forward = place_cfg["post_place_forward_m"]
                p_forward = p_place.copy()
                forward_delta_xy = place_y_xy * (-post_forward)
                p_forward[0] += forward_delta_xy[0]
                p_forward[1] += forward_delta_xy[1]
                self.get_logger().info(
                    f"[POST_PLACE_FORWARD] entlang Ablage +Y={post_forward:.3f}m, "
                    f"delta_xy=({forward_delta_xy[0]:+.4f}, {forward_delta_xy[1]:+.4f})"
                )
                jt_forward, z_forward = self._plan_with_retry(p_forward, q_place, "POST_PLACE_FORWARD")
                p_forward[2] = z_forward
                self.execute_trajectory(jt_forward)

                # 11) Vertikal nach oben wegziehen (template-spezifische Distanz)
                post_retreat = place_cfg["post_place_retreat_m"]
                p_retreat = p_forward.copy()
                p_retreat[2] += post_retreat
                self.get_logger().info(f"[RETREAT] Vertikal nach oben um {post_retreat:.3f}m")
                jt_retreat, z_retreat = self._plan_with_retry(p_retreat, q_place, "PLACE_RETREAT")
                p_retreat[2] = z_retreat
                self.execute_trajectory(jt_retreat)

            if RETURN_HOME_AFTER_GRIP and place_mode != "drop":
                try:
                    self._move_home()
                except Exception as exc:
                    self.get_logger().warn(f"[HOME] Home-Fahrt fehlgeschlagen (Ablauf bleibt OK): {exc}")

            self.get_logger().info("[OK] Ablauf komplett erfolgreich")
            self._publish_gui_status("success")

        except Exception as exc:
            import traceback
            exc_str = str(exc).lower()
            self.get_logger().error(f"[ERROR] Ablauf fehlgeschlagen: {exc}", throttle_duration_sec=1)
            self.get_logger().error(f"[ERROR] Traceback:\n{traceback.format_exc()}")

            if "tolerance" in exc_str or "path_tolerance" in exc_str:
                try:
                    gripper(self, close=False, pulse=True, pulse_time=OPEN_SECONDS)
                except Exception as g_exc:
                    self.get_logger().warn(f"[TOLERANCE] Greifer öffnen fehlgeschlagen: {g_exc}")
                try:
                    self._move_home()
                except Exception as home_exc:
                    self.get_logger().warn(f"[TOLERANCE] Rueckfahrt nach Toleranzfehler fehlgeschlagen: {home_exc}")
                self._publish_gui_status("tolerance_violation")
            elif "keine trajektorie" in exc_str or "no path" in exc_str or "planning failed" in exc_str:
                try:
                    self._move_home()
                except Exception as home_exc:
                    self.get_logger().warn(f"[HOME] Rueckfahrt nach Fehler fehlgeschlagen: {home_exc}")
                self._publish_gui_status("no_path")
            else:
                self._publish_gui_status("no_path")
        finally:
            with self.busy_lock:
                self.busy = False

    def _publish_gui_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.gui_status_pub.publish(msg)
        self.get_logger().info(f"[GUI-STATUS] {status}")

    def _on_target_pose(self, msg: PoseStamped) -> None:
        with self.count_lock:
            self.pose_received_count += 1

        base_frame, template_id = split_frame_and_template(msg.header.frame_id)
        if normalize_frame_id(msg.header.frame_id) != msg.header.frame_id:
            self.get_logger().warn(
                f"[FRAME] Normalisiere frame_id '{msg.header.frame_id}' -> '{base_frame}'"
            )

        self.get_logger().info("-------------------------------------------------------")
        self.get_logger().info(f"[EMPFANGEN] Zielpose on {TARGET_TOPIC}")
        self.get_logger().info(f"  Frame: {msg.header.frame_id}")
        self.get_logger().info(f"  Template: {template_id}")
        self.get_logger().info(
            f"  Position: x={msg.pose.position.x:.4f}, y={msg.pose.position.y:.4f}, z={msg.pose.position.z:.4f}"
        )

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

    cli = node.create_client(SetIO, "/io_and_status_controller/set_io")
    if not cli.wait_for_service(timeout_sec=timeout):
        node.get_logger().error("Service /io_and_status_controller/set_io nicht verfuegbar")
        return False

    req = SetIO.Request()
    req.fun = 1
    req.pin = int(pin)
    req.state = float(state)
    fut = cli.call_async(req)

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

    ok = bool(getattr(res, "success", True)) if res is not None else False
    if not ok:
        node.get_logger().error(f"Tool DO fehlgeschlagen: pin={req.pin} -> {req.state}; Antwort={res}")
    return ok


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
    node = UR3GripAndPlaceNode()
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
