#!/usr/bin/env python3
"""
Interactive gripper offset calibration tool.

Workflow per cycle:
1) Move robot to camera pose.
2) Wait for detected tool center pose on /tool_target_pose (+ template ID on /tool_target_template).
3) Operator moves robot manually to the desired grasp pose.
4) On Enter, read current TCP pose and compute transform:
   detected_center -> manual_grasp (translation + orientation).
5) Print copy-paste snippet for TEMPLATE_GRASP_OFFSETS.
6) On next Enter, move back to camera pose and wait for next cycle.
"""

import math
import threading
import time
from typing import Optional, Tuple

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
DEFAULT_TEMPLATE_ID = "cropv1_clean_direction"

SERVICE_TIMEOUT_SEC = 8.0
ACTION_TIMEOUT_SEC = 30.0
IK_SERVICE_TIMEOUT_SEC = 20.0
PLAN_SERVICE_TIMEOUT_SEC = 25.0
JOINT_STATE_WAIT_SEC = 10.0

GRIPPER_FIXED_ROLL = math.pi
GRIPPER_FIXED_PITCH = 0.0
TOOL_YAW_OFFSET = 0.0

# Camera pose in table coordinates
HOME_TABLE_X_M = 0.17
HOME_TABLE_Y_M = -0.04
HOME_TABLE_Z_M = 0.14
HOME_YAW_RAD = math.pi

TARGET_POSE_IS_TABLE_COORDS = True
TABLE_ORIGIN_IN_BASE_X_M = -0.15
TABLE_ORIGIN_IN_BASE_Y_M = 0.15
TABLE_ORIGIN_IN_BASE_Z_M = 0.0
TABLE_TO_BASE_X_SIGN = -1.0
TABLE_TO_BASE_Y_SIGN = -1.0
TABLE_TO_BASE_Z_SIGN = 1.0

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


def rotmat_to_rpy(r: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(r[0, 0] * r[0, 0] + r[1, 0] * r[1, 0])
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(r[2, 1], r[2, 2])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


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


class GripperOffsetCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("gripper_offset_calibration_node")

        self.group = "ur_manipulator"
        self.ee_link = "tcp"
        self.base = "base_link"
        self.action_name = "/scaled_joint_trajectory_controller/follow_joint_trajectory"

        self.cli_ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cli_plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cli_fk = self.create_client(GetPositionFK, "/compute_fk")
        self.cli_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.exec_ac = ActionClient(self, FollowJointTrajectory, self.action_name)

        for cli in [self.cli_ik, self.cli_plan, self.cli_fk, self.cli_scene]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("MoveIt Service nicht verfuegbar")
        if not self.exec_ac.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Trajectory Action Server nicht verfuegbar")

        self.js_lock = threading.Lock()
        self.latest_joint_state: Optional[JointState] = None

        self.target_lock = threading.Lock()
        self.latest_target_pose: Optional[PoseStamped] = None
        self.latest_template_id = DEFAULT_TEMPLATE_ID
        self.target_event = threading.Event()

        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(PoseStamped, TARGET_TOPIC, self._on_target_pose, qos_pose)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)

        self._shutdown = False
        self.worker_thread = threading.Thread(target=self._workflow, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("Kalibrierung gestartet")
        self.get_logger().info(f"Warte auf {TARGET_TOPIC} (frame_id='frame|template_id')")

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

    def _on_target_pose(self, msg: PoseStamped) -> None:
        frame, template_id = split_frame_and_template(msg.header.frame_id)
        msg.header.frame_id = frame
        with self.target_lock:
            self.latest_target_pose = msg
            self.latest_template_id = template_id
        self.target_event.set()

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
            raise RuntimeError(f"Kein IK-Ergebnis: {last_ik_error}")

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
        mpr.max_velocity_scaling_factor = 0.3
        mpr.max_acceleration_scaling_factor = 0.3
        mpr.allowed_planning_time = 5.0
        mpr.num_planning_attempts = 3

        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = mpr
        fut = self.cli_plan.call_async(plan_req)
        plan_res = self._wait_future_result(fut, "GetMotionPlan", PLAN_SERVICE_TIMEOUT_SEC)
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
            p_base = np.array([
                TABLE_ORIGIN_IN_BASE_X_M + (TABLE_TO_BASE_X_SIGN * p[0]),
                TABLE_ORIGIN_IN_BASE_Y_M + (TABLE_TO_BASE_Y_SIGN * p[1]),
                TABLE_ORIGIN_IN_BASE_Z_M + (TABLE_TO_BASE_Z_SIGN * p[2]),
            ], dtype=np.float64)
            q_out = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
            return p_base, q_out

        r_base_aruco = rpy_to_rotmat(ARUCO_IN_BASE_RX, ARUCO_IN_BASE_RY, ARUCO_IN_BASE_RZ)
        p_base_aruco = np.array([ARUCO_IN_BASE_X_M, ARUCO_IN_BASE_Y_M, ARUCO_IN_BASE_Z_M], dtype=np.float64)

        p_base = p_base_aruco + (r_base_aruco @ p)
        r_base_target = r_base_aruco @ r_target
        qx, qy, qz, qw = rotmat_to_quat(r_base_target)
        q_out = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return p_base, q_out

    def _move_camera_pose(self) -> None:
        q_home = rpy_to_quat(
            GRIPPER_FIXED_ROLL,
            GRIPPER_FIXED_PITCH,
            HOME_YAW_RAD + TOOL_YAW_OFFSET,
        )
        home_x = TABLE_ORIGIN_IN_BASE_X_M + (TABLE_TO_BASE_X_SIGN * HOME_TABLE_X_M)
        home_y = TABLE_ORIGIN_IN_BASE_Y_M + (TABLE_TO_BASE_Y_SIGN * HOME_TABLE_Y_M)
        home_z = TABLE_ORIGIN_IN_BASE_Z_M + (TABLE_TO_BASE_Z_SIGN * HOME_TABLE_Z_M)

        jt_home = self.plan_to_pose_quat(home_x, home_y, home_z, q_home)
        self.execute_trajectory(jt_home)

    def _get_current_tcp_pose(self) -> PoseStamped:
        req = GetPositionFK.Request()
        req.header.frame_id = self.base
        req.fk_link_names = [self.ee_link]
        req.robot_state = self.get_robot_state()

        fut = self.cli_fk.call_async(req)
        res = self._wait_future_result(fut, "GetPositionFK", SERVICE_TIMEOUT_SEC)
        if res is None or not res.pose_stamped:
            raise RuntimeError("FK lieferte keine Pose")
        return res.pose_stamped[0]

    def _wait_for_target_pose(self) -> Tuple[PoseStamped, str]:
        self.target_event.clear()
        self.get_logger().info(f"Warte auf naechste Pose auf {TARGET_TOPIC}...")
        while rclpy.ok() and not self._shutdown:
            if self.target_event.wait(timeout=0.2):
                with self.target_lock:
                    pose = self.latest_target_pose
                    template_id = self.latest_template_id
                if pose is not None:
                    return pose, template_id
        raise RuntimeError("Node wird beendet")

    def _format_pose(self, label: str, pose: PoseStamped) -> str:
        p = pose.pose.position
        q = pose.pose.orientation
        r = quat_to_rotmat(q.x, q.y, q.z, q.w)
        roll, pitch, yaw = rotmat_to_rpy(r)
        return (
            f"{label}: frame={pose.header.frame_id}, "
            f"pos=({p.x:+.5f}, {p.y:+.5f}, {p.z:+.5f}) m, "
            f"quat=({q.x:+.6f}, {q.y:+.6f}, {q.z:+.6f}, {q.w:+.6f}), "
            f"rpy=({roll:+.5f}, {pitch:+.5f}, {yaw:+.5f}) rad"
        )

    def _compute_offset(self, detected_base: PoseStamped, manual_base: PoseStamped):
        pd = np.array([
            detected_base.pose.position.x,
            detected_base.pose.position.y,
            detected_base.pose.position.z,
        ], dtype=np.float64)
        qd = detected_base.pose.orientation
        rd = quat_to_rotmat(qd.x, qd.y, qd.z, qd.w)

        pm = np.array([
            manual_base.pose.position.x,
            manual_base.pose.position.y,
            manual_base.pose.position.z,
        ], dtype=np.float64)
        qm = manual_base.pose.orientation
        rm = quat_to_rotmat(qm.x, qm.y, qm.z, qm.w)

        p_off = rd.T @ (pm - pd)
        r_off = rd.T @ rm
        qx, qy, qz, qw = rotmat_to_quat(r_off)
        rr, rp, ry = rotmat_to_rpy(r_off)

        return p_off, (qx, qy, qz, qw), (rr, rp, ry)

    def _workflow(self) -> None:
        while rclpy.ok() and not self._shutdown:
            try:
                self.get_logger().info("=== Neuer Kalibrierzyklus ===")
                self.get_logger().info("Fahre in Kameraposition...")
                self._move_camera_pose()
                self.get_logger().info("Kameraposition erreicht")

                detected_pose_raw, template_id = self._wait_for_target_pose()
                detected_base_p, detected_base_q = self._target_in_base(detected_pose_raw)

                detected_base = PoseStamped()
                detected_base.header.frame_id = self.base
                detected_base.header.stamp = self.get_clock().now().to_msg()
                detected_base.pose.position.x = float(detected_base_p[0])
                detected_base.pose.position.y = float(detected_base_p[1])
                detected_base.pose.position.z = float(detected_base_p[2])
                detected_base.pose.orientation = detected_base_q

                print("\n--- ERKANNTE POSE ---")
                print(self._format_pose("detected_raw", detected_pose_raw))
                print(self._format_pose("detected_base", detected_base))
                print(f"template_id: {template_id}")

                input("\nRoboter manuell in echte Greifpose fahren, dann Enter druecken...")
                manual_tcp = self._get_current_tcp_pose()

                print("\n--- MANUELLE GREIFPOSE (TCP) ---")
                print(self._format_pose("manual_tcp", manual_tcp))

                p_off, q_off, rpy_off = self._compute_offset(detected_base, manual_tcp)

                print("\n--- BERECHNETER OFFSET (detected_center -> grasp_tcp) ---")
                print(
                    "translation_xyz_m = "
                    f"({p_off[0]:+.6f}, {p_off[1]:+.6f}, {p_off[2]:+.6f})"
                )
                print(
                    "rotation_quat_xyzw = "
                    f"({q_off[0]:+.6f}, {q_off[1]:+.6f}, {q_off[2]:+.6f}, {q_off[3]:+.6f})"
                )
                print(
                    "rotation_rpy_rad = "
                    f"({rpy_off[0]:+.6f}, {rpy_off[1]:+.6f}, {rpy_off[2]:+.6f})"
                )
                print("\nCopy-paste fuer TEMPLATE_GRASP_OFFSETS:")
                print(
                    f'"{template_id}": {{"translation_xyz_m": '
                    f'({p_off[0]:+.6f}, {p_off[1]:+.6f}, {p_off[2]:+.6f}), '
                    f'"rotation_quat_xyzw": '
                    f'({q_off[0]:+.6f}, {q_off[1]:+.6f}, {q_off[2]:+.6f}, {q_off[3]:+.6f})}},'
                )

                input("\nEnter fuer Rueckfahrt in Kameraposition...")
            except (EOFError, KeyboardInterrupt):
                self.get_logger().info("Kalibrierung durch Benutzer beendet")
                rclpy.shutdown()
                return
            except Exception as exc:
                self.get_logger().error(f"Kalibrierzyklus fehlgeschlagen: {exc}")
                time.sleep(1.0)

    def destroy_node(self):
        self._shutdown = True
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperOffsetCalibrationNode()
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
