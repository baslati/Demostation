#!/usr/bin/env python3
"""
UR3 Provide From Storage - Umgekehrte Logik

Holt Zangen von ihren bekannten Lagerpositionen und bringt sie
zur Bereitstellungsposition (HOME-XY, z=0).

Bedienung im Terminal:
  1  ->  Breite Zange   (breitv1_clean_direction)
  2  ->  Lange Zange    (langv1_clean_direction)
  3  ->  Kurze Zange    (kurzv3_clean_direction)
  q  ->  Beenden

Ablauf:
1) Initialfahrt zur Home-Pose
2) Zeigt Auswahl-Menue im Terminal
3) Greift Zange von bekannter Lagerposition
   (breit/lang: hover -> absenken -> verschieben -> schliessen)
   (kurz: hover-Gelenkwinkel -> Greif-Gelenkwinkel -> schliessen)
4) Hebt an, faehrt zur Home-Pose (Zwischenpunkt)
5) Bringt Zange zur Bereitstellungsposition, legt ab
6) Kehrt zur Home-Pose zurueck -> Menue erscheint erneut
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
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


# Tastenbelegung im Terminal-Menue
MENU_KEYS = {
    "1": "breitv1_clean_direction",
    "2": "langv1_clean_direction",
    "3": "kurzv3_clean_direction",
}

# ---------------------------------------------------------------------------
# Lager-Positionen (entsprechen den Ablagepunkten aus ur3_grip_and_place.py)
# ---------------------------------------------------------------------------
STORAGE_PICK_CONFIG: Dict[str, Dict] = {
    "breitv1_clean_direction": {
        "pick_mode": "table",       # Tischgreif-Logik (hover -> absenken -> verschieben)
        "pick_x": 0.06,
        "pick_y": 0.175,
        "pick_z": 0.0,
        "pick_yaw": math.pi * 1.1,
        "pick_center_offset_y_m": 0.01,  # pick_center +1 cm entlang Greifer-Y verschieben
        "approach_shift_m": 0.03,   # Verschieben entlang Greifer-Y vor dem Schliessen
        "post_pick_lift_m": 0.02,   # Vertikaler Anstieg nach dem Greifen
    },
    "langv1_clean_direction": {
        "pick_mode": "table",
        "pick_x": 0.16,
        "pick_y": 0.17,
        "pick_z": 0.0,
        "pick_yaw": math.pi * 1.25,
        "approach_shift_m": 0.03,
        "post_pick_lift_m": 0.02,
    },
    "kurzv3_clean_direction": {
        "pick_mode": "joint",       # Gelenkwinkel-basiertes Greifen (schraege Halterung)
        # TODO: Mit /joint_states ausmessen und hier eintragen
        "hover_joint_config": {
            "shoulder_pan_joint":  0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_joint":         0.0,
            "wrist_1_joint":       0.0,
            "wrist_2_joint":       0.0,
            "wrist_3_joint":       0.0,
        },
        # TODO: Mit /joint_states ausmessen und hier eintragen
        "grasp_joint_config": {
            "shoulder_pan_joint":  0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_joint":         0.0,
            "wrist_1_joint":       0.0,
            "wrist_2_joint":       0.0,
            "wrist_3_joint":       0.0,
        },
        # Gelenkwinkel nach dem Greifen (Rueckzug aus der Halterung, typisch = hover_joint_config)
        # TODO: Ggf. eigene Rueckzug-Pose ausmessen; sonst None -> faehrt direkt zur Home-Pose
        "retreat_joint_config": None,
        "post_pick_lift_m": 0.05,   # nur fuer Fallback (wird bei joint-mode nicht als Kartesisch genutzt)
    },
}

DEFAULT_PICK_KEY = "breitv1_clean_direction"

# ---------------------------------------------------------------------------
# Bereitstellungsposition: HOME-XY, z=0 (auf Tischoberfläche)
# ---------------------------------------------------------------------------
TABLE_ORIGIN_IN_BASE_X_M = -0.15
TABLE_ORIGIN_IN_BASE_Y_M =  0.15
TABLE_ORIGIN_IN_BASE_Z_M =  0.0
TABLE_TO_BASE_X_SIGN = -1.0
TABLE_TO_BASE_Y_SIGN = -1.0
TABLE_TO_BASE_Z_SIGN =  1.0

HOME_TABLE_X_M = 0.17
HOME_TABLE_Y_M = -0.035
HOME_TABLE_Z_M = 0.14
HOME_ROLL_RAD  = math.pi
HOME_PITCH_RAD = 0.0
HOME_YAW_RAD   = math.pi - 0.02

PROVIDE_TABLE_X_M =  HOME_TABLE_X_M   # gleiche XY wie Home
PROVIDE_TABLE_Y_M =  HOME_TABLE_Y_M
PROVIDE_TABLE_Z_M =  0.0              # auf Tischoberfläche
PROVIDE_YAW_RAD   =  HOME_YAW_RAD

# ---------------------------------------------------------------------------
# Allgemeine Parameter
# ---------------------------------------------------------------------------
SERVICE_TIMEOUT_SEC      = 8.0
ACTION_TIMEOUT_SEC       = 30.0
IK_SERVICE_TIMEOUT_SEC   = 20.0
JOINT_STATE_WAIT_SEC     = 10.0
STARTUP_MOVE_HOME        = True

HOLD_SECONDS  = 0.5
OPEN_SECONDS  = 0.5

HOVER_ABOVE_PICK_M      = 0.03
PICK_Z_M                = 0.003   # Greifhoehe ueber Tisch (base-frame)
PROVIDE_HOVER_ABOVE_M   = 0.05    # Hover ueber Bereitstellungsposition
PROVIDE_Z_M             = 0.003   # Ablegehoehe (Bereitstellung)
PROVIDE_RETREAT_UP_M    = 0.03    # Vertikaler Rueckzug nach dem Ablegen

GRIPPER_FIXED_ROLL  = math.pi
GRIPPER_FIXED_PITCH = 0.0
TOOL_YAW_OFFSET     = 0.0

MIN_TARGET_Z_IN_BASE_M      = 0.0032
PLANNING_Z_RETRY_STEPS_M    = (0.0, 0.008, 0.015)
APPROACH_HOVER_YAW_RETRY_DEG   = (0.0, 5.0, -5.0, 10.0, -10.0)

FJT_ERROR_TEXT = {
     0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -2: "INVALID_JOINTS",
    -3: "OLD_HEADER_TIMESTAMP",
    -4: "PATH_TOLERANCE_VIOLATED",
    -5: "GOAL_TOLERANCE_VIOLATED",
}


# ---------------------------------------------------------------------------
# Hilfsfunktionen (identisch mit ur3_grip_and_place.py)
# ---------------------------------------------------------------------------

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
    x2 = qx + qx; y2 = qy + qy; z2 = qz + qz
    xx = qx * x2; yy = qy * y2; zz = qz * z2
    xy = qx * y2; xz = qx * z2; yz = qy * z2
    wx = qw * x2; wy = qw * y2; wz = qw * z2
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy        ],
        [xy + wz,         1.0 - (xx + zz), yz - wx        ],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ], dtype=np.float64)


def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class UR3ProvideFromStorageNode(Node):
    def __init__(self) -> None:
        super().__init__("ur3_provide_from_storage_node")

        self.get_logger().info("=======================================================")
        self.get_logger().info("  UR3 PROVIDE FROM STORAGE NODE START")
        self.get_logger().info("=======================================================")

        self.group      = "ur_manipulator"
        self.ee_link    = "tcp"
        self.base       = "base_link"
        self.action_name = "/scaled_joint_trajectory_controller/follow_joint_trajectory"

        self.get_logger().info(
            f"[INIT] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'nicht gesetzt')}"
        )

        self.cli_ik    = self.create_client(GetPositionIK,    "/compute_ik")
        self.cli_plan  = self.create_client(GetMotionPlan,    "/plan_kinematic_path")
        self.cli_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.exec_ac   = ActionClient(self, FollowJointTrajectory, self.action_name)

        for cli in [self.cli_ik, self.cli_plan, self.cli_scene]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("MoveIt Service nicht verfuegbar")

        if not self.exec_ac.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Trajectory Action Server nicht verfuegbar")

        self.get_logger().info("[INIT] Alle Services gefunden")

        self.busy_lock  = threading.Lock()
        self.busy       = False
        self.js_lock    = threading.Lock()
        self.latest_joint_state = None
        self.startup_done = False
        self.ready_event = threading.Event()  # wird gesetzt sobald Initialfahrt fertig

        self.gui_status_pub = self.create_publisher(String, "/gui/robot_status", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(String, "/gui/plier_selection", self._on_plier_selection, 10)
        self.create_timer(0.5, self._startup_home_once)

        self.get_logger().info("=======================================================")
        self.get_logger().info("  NODE BEREIT - INITIALFAHRT LAEUFT...")
        self.get_logger().info("=======================================================")

    # ------------------------------------------------------------------
    # Joint-State Hilfsmethoden
    # ------------------------------------------------------------------

    def _publish_gui_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.gui_status_pub.publish(msg)
        self.get_logger().info(f"[GUI-STATUS] {status}")

    def _on_plier_selection(self, msg: String) -> None:
        template_id = msg.data.strip()
        if template_id not in STORAGE_PICK_CONFIG:
            self.get_logger().warn(f"[GUI] Unbekannter Template-Name: '{template_id}'")
            return
        with self.busy_lock:
            if self.busy:
                self.get_logger().warn("[GUI] Roboter ist beschäftigt – Anfrage ignoriert")
                return
        self.get_logger().info(f"[GUI] Starte: {template_id}")
        self._publish_gui_status("executing")
        self.request_pick(template_id)

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
                self.get_logger().warn("[STARTUP] Keine /joint_states – Initialfahrt uebersprungen")
            else:
                try:
                    self._move_home()
                    self.get_logger().info("[STARTUP] Initialfahrt erfolgreich")
                except Exception as exc:
                    self.get_logger().error(f"[STARTUP] Initialfahrt fehlgeschlagen: {exc}")
            self.ready_event.set()

        threading.Thread(target=worker, daemon=True).start()

    # ------------------------------------------------------------------
    # Robot-State
    # ------------------------------------------------------------------

    def get_robot_state(self) -> RobotState:
        try:
            req = GetPlanningScene.Request()
            req.components = PlanningSceneComponents(
                components=PlanningSceneComponents.ROBOT_STATE
            )
            fut = self.cli_scene.call_async(req)
            res = self._wait_future_result(fut, "GetPlanningScene", SERVICE_TIMEOUT_SEC)
            if res is not None and res.scene is not None:
                return res.scene.robot_state
        except Exception as exc:
            self.get_logger().warn(
                f"[STATE] GetPlanningScene fehlgeschlagen, nutze /joint_states: {exc}"
            )

        with self.js_lock:
            js = self.latest_joint_state
        if js is None or not js.name:
            raise RuntimeError("Noch keine /joint_states erhalten")

        rs = RobotState()
        rs.joint_state.name     = list(js.name)
        rs.joint_state.position = list(js.position)
        rs.joint_state.velocity = list(js.velocity)
        rs.joint_state.effort   = list(js.effort)
        rs.joint_state.header   = js.header
        return rs

    # ------------------------------------------------------------------
    # Future-Hilfsmethode
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Planung und Ausfuehrung
    # ------------------------------------------------------------------

    def plan_to_pose_quat(self, x: float, y: float, z: float, q: Quaternion) -> JointTrajectory:
        pose = PoseStamped()
        pose.header.frame_id = self.base
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation = q

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name    = self.group
        ik_req.ik_request.robot_state   = self.get_robot_state()
        ik_req.ik_request.pose_stamped  = pose
        ik_req.ik_request.ik_link_name  = self.ee_link
        ik_req.ik_request.timeout       = Duration(sec=2, nanosec=0)

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
        for name, pos in zip(
            ik_res.solution.joint_state.name, ik_res.solution.joint_state.position
        ):
            jc = JointConstraint()
            jc.joint_name    = name
            jc.position      = pos
            jc.tolerance_above = 1e-3
            jc.tolerance_below = 1e-3
            jc.weight        = 1.0
            constraints.joint_constraints.append(jc)

        mpr = MotionPlanRequest()
        mpr.group_name                     = self.group
        mpr.goal_constraints               = [constraints]
        mpr.start_state                    = self.get_robot_state()
        mpr.max_velocity_scaling_factor    = 0.4
        mpr.max_acceleration_scaling_factor = 0.4
        mpr.allowed_planning_time          = 5.0
        mpr.num_planning_attempts          = 3

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
            jc.joint_name      = name
            jc.position        = float(pos)
            jc.tolerance_above = 1e-3
            jc.tolerance_below = 1e-3
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        mpr = MotionPlanRequest()
        mpr.group_name                     = self.group
        mpr.goal_constraints               = [constraints]
        mpr.start_state                    = self.get_robot_state()
        mpr.max_velocity_scaling_factor    = 0.4
        mpr.max_acceleration_scaling_factor = 0.4
        mpr.allowed_planning_time          = 5.0
        mpr.num_planning_attempts          = 3

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
        goal.trajectory          = jt
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        fut = self.exec_ac.send_goal_async(goal)
        gh  = self._wait_future_result(fut, "SendGoal", ACTION_TIMEOUT_SEC)
        if gh is None or not gh.accepted:
            raise RuntimeError("Goal abgelehnt")
        res_fut = gh.get_result_async()
        result  = self._wait_future_result(res_fut, "GetResult", ACTION_TIMEOUT_SEC)
        if result is None:
            raise RuntimeError("Ausfuehrung fehlgeschlagen: kein Ergebnis")
        error_code = int(getattr(result.result, "error_code", 0))
        error_str  = str(getattr(result.result, "error_string", "")).strip()
        if error_code != 0:
            code_label = FJT_ERROR_TEXT.get(error_code, "UNBEKANNT")
            details    = f"error_code={error_code} ({code_label})"
            if error_str:
                details += f", error_string='{error_str}'"
            raise RuntimeError(f"Ausfuehrung fehlgeschlagen: {details}")

    # ------------------------------------------------------------------
    # Koordinaten-Hilfsmethoden
    # ------------------------------------------------------------------

    def _table_to_base(self, x: float, y: float, z: float) -> np.ndarray:
        return np.array([
            TABLE_ORIGIN_IN_BASE_X_M + TABLE_TO_BASE_X_SIGN * x,
            TABLE_ORIGIN_IN_BASE_Y_M + TABLE_TO_BASE_Y_SIGN * y,
            TABLE_ORIGIN_IN_BASE_Z_M + TABLE_TO_BASE_Z_SIGN * z,
        ], dtype=np.float64)

    def _plan_with_retry(
        self, p: np.ndarray, q_target: Quaternion, label: str
    ) -> Tuple[JointTrajectory, float]:
        planning_errors = []
        for dz in PLANNING_Z_RETRY_STEPS_M:
            z_try = float(p[2] + dz)
            self.get_logger().info(f"[PLANNING] {label}: z={z_try:.4f} (dz={dz:+.3f})")
            try:
                jt = self.plan_to_pose_quat(float(p[0]), float(p[1]), z_try, q_target)
                return jt, z_try
            except Exception as exc:
                planning_errors.append(str(exc))

            if label == "PICK_HOVER":
                base_yaw = yaw_from_quat(q_target)
                for yaw_deg in APPROACH_HOVER_YAW_RETRY_DEG[1:]:
                    q_retry = rpy_to_quat(
                        GRIPPER_FIXED_ROLL,
                        GRIPPER_FIXED_PITCH,
                        base_yaw + math.radians(yaw_deg),
                    )
                    try:
                        jt = self.plan_to_pose_quat(float(p[0]), float(p[1]), z_try, q_retry)
                        return jt, z_try
                    except Exception as exc:
                        planning_errors.append(str(exc))

        raise RuntimeError(
            f"{label}: keine Trajektorie; letzte Ursache="
            f"{planning_errors[-1] if planning_errors else 'unbekannt'}"
        )

    def _move_home(self) -> None:
        q_home = rpy_to_quat(HOME_ROLL_RAD, HOME_PITCH_RAD, HOME_YAW_RAD + TOOL_YAW_OFFSET)
        home   = self._table_to_base(HOME_TABLE_X_M, HOME_TABLE_Y_M, HOME_TABLE_Z_M)
        jt_home, _ = self._plan_with_retry(home, q_home, "HOME")
        self.execute_trajectory(jt_home)
        self.get_logger().info("[HOME] Zur Home-Pose gefahren")

    # ------------------------------------------------------------------
    # Greifsequenz: Tisch (breit / lang)
    # ------------------------------------------------------------------

    def _do_table_pick(self, pick_cfg: Dict) -> np.ndarray:
        """
        Greift Zange von bekannter Tischposition.
        Hover und Abstieg erfolgen direkt ueber dem versetzten Greifpunkt
        (approach_shift_m entlang Greifer-Y), kein horizontales Verschieben.
        Rueckgabe: Position nach dem Greifen (in base-frame).
        """
        q_pick = rpy_to_quat(
            GRIPPER_FIXED_ROLL,
            GRIPPER_FIXED_PITCH,
            pick_cfg["pick_yaw"] + TOOL_YAW_OFFSET,
        )

        # Y-Richtung des Greifers in der XY-Ebene
        r_pick = quat_to_rotmat(q_pick.x, q_pick.y, q_pick.z, q_pick.w)
        gripper_y_in_base = r_pick[:, 1]
        gripper_y_xy = np.array(
            [gripper_y_in_base[0], gripper_y_in_base[1], 0.0], dtype=np.float64
        )
        norm_xy = float(np.linalg.norm(gripper_y_xy))
        if norm_xy < 1e-9:
            raise RuntimeError("Greifer-Y kann nicht in XY-Ebene projiziert werden")
        gripper_y_xy /= norm_xy

        shift = pick_cfg.get("approach_shift_m", 0.05)
        delta_xy = gripper_y_xy * shift

        p_pick_center = self._table_to_base(
            pick_cfg["pick_x"], pick_cfg["pick_y"], PICK_Z_M
        )
        if p_pick_center[2] < MIN_TARGET_Z_IN_BASE_M:
            p_pick_center[2] = MIN_TARGET_Z_IN_BASE_M

        # Optionaler Versatz des pick_center entlang Greifer-Y
        center_offset = pick_cfg.get("pick_center_offset_y_m", 0.0)
        if abs(center_offset) > 1e-9:
            p_pick_center[0] += gripper_y_xy[0] * center_offset
            p_pick_center[1] += gripper_y_xy[1] * center_offset
            self.get_logger().info(
                f"[PICK] pick_center +Y={center_offset*100:.0f}cm Versatz angewendet: "
                f"({p_pick_center[0]:.4f}, {p_pick_center[1]:.4f})"
            )

        # Startpunkt = pick_center - shift*Y  (hinter dem Greifpunkt)
        p_start = p_pick_center.copy()
        p_start[0] -= delta_xy[0]
        p_start[1] -= delta_xy[1]
        self.get_logger().info(
            f"[PICK] Anfahrpunkt (pick_center -Y): "
            f"({p_start[0]:.4f}, {p_start[1]:.4f}, {p_start[2]:.4f})"
        )

        # 1) Hover ueber Anfahrpunkt (pick_center - Y)
        p_hover = p_start.copy()
        p_hover[2] += HOVER_ABOVE_PICK_M
        self.get_logger().info(
            f"[PICK_HOVER] Ziel=({p_hover[0]:.4f}, {p_hover[1]:.4f}, {p_hover[2]:.4f})"
        )
        jt_hover, z_hover = self._plan_with_retry(p_hover, q_pick, "PICK_HOVER")
        p_hover[2] = z_hover
        self.execute_trajectory(jt_hover)

        # 2) Vertikal absenken auf Greifhoehe
        p_descend = p_start.copy()
        p_descend[2] = min(p_hover[2], p_descend[2])
        self.get_logger().info(
            f"[PICK_DESCEND] Ziel=({p_descend[0]:.4f}, {p_descend[1]:.4f}, {p_descend[2]:.4f})"
        )
        jt_desc, z_desc = self._plan_with_retry(p_descend, q_pick, "PICK_DESCEND")
        p_descend[2] = z_desc
        self.execute_trajectory(jt_desc)

        # 3) Horizontal verschieben +Y auf pick_center (Backe unter den Griff schieben)
        p_shift = p_descend.copy()
        p_shift[0] += delta_xy[0]
        p_shift[1] += delta_xy[1]
        self.get_logger().info(
            f"[PICK_SHIFT] +Y={shift:.3f}m -> ({p_shift[0]:.4f}, {p_shift[1]:.4f}, {p_shift[2]:.4f})"
        )
        jt_shift, z_shift = self._plan_with_retry(p_shift, q_pick, "PICK_SHIFT")
        p_shift[2] = z_shift
        self.execute_trajectory(jt_shift)

        # 4) Greifer schliessen
        self.get_logger().info("[GRIPPER] Schliessen...")
        if not set_tool_do(self, 16, 1.0):
            raise RuntimeError("Greifer CLOSE (Pin 16=1) fehlgeschlagen")
        time.sleep(HOLD_SECONDS)
        self.get_logger().info("[GRIPPER] Geschlossen")

        # 5) Vertikal anheben
        lift = pick_cfg.get("post_pick_lift_m", 0.02)
        p_lift = p_shift.copy()
        p_lift[2] += lift
        self.get_logger().info(
            f"[PICK_LIFT] Hebe um {lift:.3f}m auf z={p_lift[2]:.4f}"
        )
        jt_lift, _ = self._plan_with_retry(p_lift, q_pick, "PICK_LIFT")
        self.execute_trajectory(jt_lift)

        return p_lift

    # ------------------------------------------------------------------
    # Greifsequenz: Gelenkwinkel (kurz / schraege Halterung)
    # ------------------------------------------------------------------

    def _do_joint_pick(self, pick_cfg: Dict) -> None:
        """
        Greift Zange aus schraeger Halterung via vorgegebener Gelenkwinkel.
        """
        hover_joints  = pick_cfg["hover_joint_config"]
        grasp_joints  = pick_cfg["grasp_joint_config"]
        retreat_joints = pick_cfg.get("retreat_joint_config")

        # 1) Hover-Position anfahren
        self.get_logger().info("[JOINT_PICK] Fahre zu Hover-Position (Joint-Space)...")
        jt_hover = self.plan_to_joint_config(hover_joints)
        self.execute_trajectory(jt_hover)

        # 2) Greif-Position anfahren
        self.get_logger().info("[JOINT_PICK] Fahre zu Greif-Position (Joint-Space)...")
        jt_grasp = self.plan_to_joint_config(grasp_joints)
        self.execute_trajectory(jt_grasp)

        # 3) Greifer schliessen
        self.get_logger().info("[GRIPPER] Schliessen...")
        if not set_tool_do(self, 16, 1.0):
            raise RuntimeError("Greifer CLOSE (Pin 16=1) fehlgeschlagen")
        time.sleep(HOLD_SECONDS)
        self.get_logger().info("[GRIPPER] Geschlossen")

        # 4) Rueckzug: Hover-Pose oder eigene Rueckzugpose
        if retreat_joints is not None:
            self.get_logger().info("[JOINT_PICK] Fahre zu Rueckzug-Position (Joint-Space)...")
            jt_retreat = self.plan_to_joint_config(retreat_joints)
            self.execute_trajectory(jt_retreat)
        else:
            self.get_logger().info("[JOINT_PICK] Rueckzug zur Hover-Position...")
            jt_hover2 = self.plan_to_joint_config(hover_joints)
            self.execute_trajectory(jt_hover2)

    # ------------------------------------------------------------------
    # Ablegen an der Bereitstellungsposition
    # ------------------------------------------------------------------

    def _do_provide(self) -> None:
        q_provide = rpy_to_quat(
            GRIPPER_FIXED_ROLL, GRIPPER_FIXED_PITCH, PROVIDE_YAW_RAD + TOOL_YAW_OFFSET
        )
        p_provide = self._table_to_base(PROVIDE_TABLE_X_M, PROVIDE_TABLE_Y_M, PROVIDE_Z_M)
        if p_provide[2] < MIN_TARGET_Z_IN_BASE_M:
            p_provide[2] = MIN_TARGET_Z_IN_BASE_M

        # 1) Hover ueber Bereitstellungsposition
        p_prov_hover = p_provide.copy()
        p_prov_hover[2] += PROVIDE_HOVER_ABOVE_M
        self.get_logger().info(
            f"[PROVIDE_HOVER] Ziel=({p_prov_hover[0]:.4f}, {p_prov_hover[1]:.4f}, "
            f"{p_prov_hover[2]:.4f})"
        )
        jt_prov_hover, z_ph = self._plan_with_retry(p_prov_hover, q_provide, "PROVIDE_HOVER")
        p_prov_hover[2] = z_ph
        self.execute_trajectory(jt_prov_hover)

        # 2) Absenken auf Ablegehoehe
        self.get_logger().info(
            f"[PROVIDE_DESCEND] Ziel=({p_provide[0]:.4f}, {p_provide[1]:.4f}, "
            f"{p_provide[2]:.4f})"
        )
        jt_prov_desc, z_pd = self._plan_with_retry(p_provide, q_provide, "PROVIDE_DESCEND")
        p_provide[2] = z_pd
        self.execute_trajectory(jt_prov_desc)

        # 3) Greifer oeffnen
        self.get_logger().info("[GRIPPER] Oeffnen...")
        if not set_tool_do(self, 16, 0.0):
            raise RuntimeError("Greifer OPEN (Pin 16=0) fehlgeschlagen")
        gripper(self, close=False, pulse=True, pulse_time=OPEN_SECONDS)
        self.get_logger().info("[GRIPPER] Offen")

        # 4) Vertikal wegziehen
        p_retreat = p_provide.copy()
        p_retreat[2] += PROVIDE_RETREAT_UP_M
        self.get_logger().info(
            f"[PROVIDE_RETREAT] Hebe um {PROVIDE_RETREAT_UP_M:.3f}m auf z={p_retreat[2]:.4f}"
        )
        jt_retreat, _ = self._plan_with_retry(p_retreat, q_provide, "PROVIDE_RETREAT")
        self.execute_trajectory(jt_retreat)

    # ------------------------------------------------------------------
    # Hauptablauf
    # ------------------------------------------------------------------

    def _process_pick_request(self, template_id: str) -> None:
        try:
            pick_cfg = STORAGE_PICK_CONFIG.get(template_id)
            if pick_cfg is None:
                self.get_logger().error(
                    f"[ERROR] Unbekannte Template-ID: '{template_id}'. "
                    f"Gueltig: {list(STORAGE_PICK_CONFIG.keys())}"
                )
                return

            self.get_logger().info(f"[START] Bereitstelle Zange: {template_id}")
            self.get_logger().info(
                f"[PICK] Lagerposition: mode={pick_cfg['pick_mode']}"
            )

            pick_mode = pick_cfg["pick_mode"]

            if pick_mode == "table":
                # Tisch-Greifsequenz (breit / lang)
                self._do_table_pick(pick_cfg)

                # Zwischenpunkt: Home-Pose (sichererer Uebergang)
                self.get_logger().info("[TRANSIT] Fahre zur Home-Pose als Zwischenpunkt")
                self._move_home()

            elif pick_mode == "joint":
                # Gelenkwinkel-Greifsequenz (kurz / schraege Halterung)
                self._do_joint_pick(pick_cfg)

                # Nach Gelenkwinkel-Pick direkt zur Home-Pose
                self.get_logger().info("[TRANSIT] Fahre zur Home-Pose nach Joint-Pick")
                self._move_home()

            else:
                raise RuntimeError(f"Unbekannter pick_mode: '{pick_mode}'")

            # Bereitstellen
            self.get_logger().info("[PROVIDE] Fahre zur Bereitstellungsposition")
            self._do_provide()

            # Zurueck zur Home-Pose
            self.get_logger().info("[HOME] Fahre zur Home-Pose")
            self._move_home()

            self.get_logger().info(f"[OK] Zange '{template_id}' erfolgreich bereitgestellt")
            self._publish_gui_status("success")

        except Exception as exc:
            import traceback
            self.get_logger().error(f"[ERROR] Ablauf fehlgeschlagen: {exc}")
            self.get_logger().error(f"[ERROR] Traceback:\n{traceback.format_exc()}")
            self._publish_gui_status("failed")
        finally:
            with self.busy_lock:
                self.busy = False

    def request_pick(self, template_id: str) -> bool:
        """Startet Greif-Sequenz falls Roboter frei. Rueckgabe: True wenn gestartet."""
        with self.busy_lock:
            if self.busy:
                return False
            self.busy = True
        threading.Thread(
            target=self._process_pick_request, args=(template_id,), daemon=True
        ).start()
        return True


# ---------------------------------------------------------------------------
# Greifer-Hilfsfunktionen (identisch mit ur3_grip_and_place.py)
# ---------------------------------------------------------------------------

def set_tool_do(node: Node, pin: int, state: float, timeout: float = 5.0) -> bool:
    from ur_msgs.srv import SetIO

    cli = node.create_client(SetIO, "/io_and_status_controller/set_io")
    if not cli.wait_for_service(timeout_sec=timeout):
        node.get_logger().error("Service /io_and_status_controller/set_io nicht verfuegbar")
        return False

    req = SetIO.Request()
    req.fun   = 1
    req.pin   = int(pin)
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
        node.get_logger().error(
            f"Tool DO fehlgeschlagen: pin={req.pin} -> {req.state}; Antwort={res}"
        )
    return ok


def gripper(
    node: Node,
    close: bool = True,
    pin_close: int = 16,
    pin_open: int = 17,
    pulse_time: float = 1.0,
    pulse: bool = True,
    active_high: bool = True,
) -> None:
    pin = pin_close if close else pin_open
    on  = 1.0 if active_high else 0.0
    off = 0.0 if active_high else 1.0
    if not set_tool_do(node, pin, on):
        raise RuntimeError("Tool DO konnte nicht eingeschaltet werden")
    if pulse:
        time.sleep(max(0.0, pulse_time))
        if not set_tool_do(node, pin, off):
            raise RuntimeError("Tool DO konnte nicht ausgeschaltet werden")


# ---------------------------------------------------------------------------
# Terminal-Menue
# ---------------------------------------------------------------------------

def _print_menu() -> None:
    print("\n" + "=" * 51)
    print("  ZANGE BEREITSTELLEN")
    print("=" * 51)
    print("  [1]  Breite Zange  (breitv1_clean_direction)")
    print("  [2]  Lange Zange   (langv1_clean_direction)")
    print("  [3]  Kurze Zange   (kurzv3_clean_direction)")
    print("  [q]  Beenden")
    print("=" * 51)
    print("  Auswahl: ", end="", flush=True)


def _menu_loop(node: UR3ProvideFromStorageNode) -> None:
    print("\nWarte auf Initialfahrt...")
    node.ready_event.wait()
    _print_menu()

    while rclpy.ok():
        try:
            key = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if key == "q":
            print("Beende...")
            break

        template_id = MENU_KEYS.get(key)
        if template_id is None:
            print(f"  Unbekannte Eingabe: '{key}'. Bitte 1, 2, 3 oder q druecken.")
            print("  Auswahl: ", end="", flush=True)
            continue

        with node.busy_lock:
            if node.busy:
                print("  Roboter ist gerade in Bewegung – bitte warten.")
                print("  Auswahl: ", end="", flush=True)
                continue

        print(f"  -> Starte: {template_id}")
        node.request_pick(template_id)

        # Warten bis Bewegung fertig, dann Menue erneut anzeigen
        while True:
            time.sleep(0.2)
            with node.busy_lock:
                if not node.busy:
                    break
        _print_menu()

    rclpy.shutdown()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = UR3ProvideFromStorageNode()
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
