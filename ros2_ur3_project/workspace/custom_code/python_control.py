#!/usr/bin/env python3

import os
import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from builtin_interfaces.msg import Duration

from moveit_msgs.srv import GetPlanningScene, GetMotionPlan, GetPositionIK, GetPositionFK
from moveit_msgs.msg import (
    PlanningSceneComponents, MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, RobotState
)
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from shape_msgs.msg import SolidPrimitive
from example_interfaces.srv import Trigger, SetBool
from geometry_msgs.srv import Pose as PoseSrv


# ---------- Helpers ----------
def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

def shortest_angular_distance(a: float, b: float) -> float:
    d = (b - a) % (2.0 * math.pi)
    return d - 2.0 * math.pi if d > math.pi else d

def wrap_goal_to_nearest(cur: float, goal: float) -> float:
    return cur + shortest_angular_distance(cur, goal)

def rad_list_to_deg(vals): return [round(v * 180.0 / math.pi, 2) for v in vals]


class URMotionNode(Node):
    def __init__(self):
        super().__init__("ur_motion_node")

        # ---- Parameter ----
        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("ee_link", "tool0")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("action_name", "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("pos_tol", 0.002)
        self.declare_parameter("rot_tol", 0.02)
        self.declare_parameter("planning_time", 2.0)
        self.declare_parameter("planner_id", "RRTkConfigDefault")
        self.declare_parameter("vel_scale", 0.5)
        self.declare_parameter("acc_scale", 0.5)
        self.declare_parameter("execute", True)
        self.declare_parameter("joint_order", [
            "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
            "wrist_1_joint","wrist_2_joint","wrist_3_joint"
        ])

        # Werte lesen
        self.group         = self.get_parameter("planning_group").value
        self.ee_link       = self.get_parameter("ee_link").value
        self.base          = self.get_parameter("base_frame").value
        self.action_name   = self.get_parameter("action_name").value
        self.pos_tol       = float(self.get_parameter("pos_tol").value)
        self.rot_tol       = float(self.get_parameter("rot_tol").value)
        self.planning_time = float(self.get_parameter("planning_time").value)
        self.planner_id    = self.get_parameter("planner_id").value
        self.vel_scale     = float(self.get_parameter("vel_scale").value)
        self.acc_scale     = float(self.get_parameter("acc_scale").value)
        self.execute       = bool(self.get_parameter("execute").value)
        self.joint_order   = list(self.get_parameter("joint_order").value)

        self.get_logger().info(
            "Konfiguration:\n"
            f"  group='{self.group}', ee_link='{self.ee_link}', base='{self.base}'\n"
            f"  action='{self.action_name}', execute={self.execute}\n"
            f"  planner_id='{self.planner_id}', t_plan={self.planning_time:.2f}s, "
            f"vel_scale={self.vel_scale}, acc_scale={self.acc_scale}\n"
            f"  REST: http://{self.rest_host}:{self.rest_port}"
        )

        # ---- Infra (Reentrant + MultiThread) ----
        self.cb = ReentrantCallbackGroup()

        # **ABSOLUTE** Servicenamen verwenden
        self.cli_scene = self.create_client(GetPlanningScene, "/get_planning_scene", callback_group=self.cb)
        self.cli_plan  = self.create_client(GetMotionPlan,   "/plan_kinematic_path", callback_group=self.cb)
        self.cli_ik    = self.create_client(GetPositionIK,   "/compute_ik", callback_group=self.cb)
        self.cli_fk    = self.create_client(GetPositionFK,   "/compute_fk", callback_group=self.cb)

        # Auf Services warten (Timeout erhöht)
        for cli, name in [(self.cli_scene, "/get_planning_scene"),
                          (self.cli_plan,  "/plan_kinematic_path"),
                          (self.cli_ik,    "/compute_ik"),
                          (self.cli_fk,    "/compute_fk")]:
            self.get_logger().debug(f"Warte auf Service '{name}' ...")
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"Service '{name}' nicht verfügbar. Läuft move_group?")
                raise SystemExit(1)
        self.get_logger().info("MoveIt-Services erreichbar.")

        self.exec_ac = ActionClient(self, FollowJointTrajectory, self.action_name, callback_group=self.cb)
        self.get_logger().debug(f"Warte auf Action-Server '{self.action_name}' ...")
        if not self.exec_ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Action-Server '{self.action_name}' nicht erreichbar.")
            raise SystemExit(1)
        self.get_logger().info("FollowJointTrajectory-Action erreichbar.")

        # ---- ROS Services ----
        self.srv_get_pose = self.create_service(
            Trigger,
            "get_current_pose",
            self.handle_get_current_pose
        )
        self.srv_move_pose = self.create_service(
            PoseSrv,
            "move_to_pose",
            self.handle_move_to_pose
        )
        self.get_logger().info("ROS-Services bereit: get_current_pose, move_to_pose")

    # ---------- Utils ----------
    def _timed_call(self, label: str, client, req, timeout_s: float = 30.0):
        """Service-Call mit garantiertem Spin bis Antwort (vermeidet 30s-Timeouts)."""
        t0 = time.monotonic()
        fut = client.call_async(req)
        # **WICHTIG**: aktiv auf das Future spinnen, sonst wird Antwort nicht verarbeitet
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        dt = (time.monotonic() - t0) * 1000.0
        if not fut.done() or fut.result() is None:
            self.get_logger().error(f"{label}: Timeout/Fehler nach {dt:.1f} ms")
            raise RuntimeError(f"{label}: Timeout/Fehler")
        self.get_logger().debug(f"{label}: ok ({dt:.1f} ms)")
        return fut.result()

    def current_robot_state(self) -> RobotState:
        req = GetPlanningScene.Request()
        req.components = PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE)
        res = self._timed_call("get_planning_scene(ROBOT_STATE)", self.cli_scene, req)
        return res.scene.robot_state

    def _traj_stats(self, traj: JointTrajectory):
        if not traj.points:
            return 0.0, 0
        last = traj.points[-1].time_from_start
        return float(last.sec) + last.nanosec * 1e-9, len(traj.points)

    def _log_traj(self, traj: JointTrajectory, label="Traj"):
        dur, n = self._traj_stats(traj)
        self.get_logger().info(f"{label}: {n} Punkte, Dauer ~{dur:.3f} s, joints={traj.joint_names}")
        if n and (traj.points[0].time_from_start.sec == 0 and traj.points[0].time_from_start.nanosec == 0):
            self.get_logger().warn("Erster Punkt t=0 — Retiming prüfen (MoveIt-Adapter AddTimeParameterization).")

    # ---------- Execution ----------
    def _feedback_cb(self, feedback_msg):
        now = time.monotonic()
        if now - self._last_feedback_log > 0.5:
            fb = feedback_msg.feedback
            try:
                t_des = fb.desired.time_from_start.sec + fb.desired.time_from_start.nanosec * 1e-9
                t_act = fb.actual.time_from_start.sec + fb.actual.time_from_start.nanosec * 1e-9
                self.get_logger().debug(f"Feedback: desired t={t_des:.3f}s, actual t={t_act:.3f}s")
            except Exception:
                self.get_logger().debug("Feedback erhalten.")
            self._last_feedback_log = now

    _last_feedback_log = 0.0  # init

    def execute_trajectory(self, traj: JointTrajectory) -> bool:
        # t=0 Punkt entfernen (einige Controller mögen das nicht)
        if traj.points:
            t0 = traj.points[0].time_from_start
            if t0.sec == 0 and t0.nanosec == 0:
                self.get_logger().warn("Erster Trajektorienpunkt hat t=0 -> entferne Punkt[0].")
                traj.points.pop(0)

        if not traj.points:
            self.get_logger().error("Leere/unzeitparametrisierte Trajektorie – Abbruch.")
            return False

        self._log_traj(traj, label="Ausführungstrajektorie")

        if not bool(self.get_parameter("execute").value):
            self.get_logger().warn("execute==False -> Nur Planung/Logging, KEINE Ausführung.")
            return True

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

        self.get_logger().info("Sende FollowJointTrajectory-Goal ...")
        send_future = self.exec_ac.send_goal_async(goal, feedback_callback=self._feedback_cb)
        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < 15.0 and not send_future.done():
            time.sleep(0.01)
        gh = send_future.result()
        if gh is None or (hasattr(gh, "accepted") and not gh.accepted):
            self.get_logger().error("FollowJointTrajectory: Goal abgelehnt/kein Handle.")
            return False

        self.get_logger().info("Goal akzeptiert, warte auf Result ...")
        res_future = gh.get_result_async()
        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < 120.0 and not res_future.done():
            time.sleep(0.02)
        result = res_future.result()
        if result is None:
            self.get_logger().error("FollowJointTrajectory: Kein Result erhalten (Timeout).")
            return False

        ec = getattr(result.result, "error_code", 0)
        es = getattr(result.result, "error_string", "")
        if ec != 0:
            self.get_logger().error(f"Ausführung fehlgeschlagen (error_code={ec}, '{es}')")
            return False

        self.get_logger().info("Trajektorie erfolgreich ausgeführt.")
        return True

    # ---------- Planning ----------
    def plan_to_joint_goal(self, joint_positions_by_name: Dict[str, float]) -> JointTrajectory:
        self.get_logger().info("Planung (Joint-Ziel) gestartet ...")
        cur = self.current_robot_state()
        cur_map = {n: p for n, p in zip(cur.joint_state.name, cur.joint_state.position)}

        # wrap auf nächstliegende Winkel
        wrapped: Dict[str, float] = {}
        for jn in self.joint_order:
            if jn not in joint_positions_by_name:
                raise ValueError(f"Joint '{jn}' fehlt im Ziel.")
            wrapped[jn] = wrap_goal_to_nearest(cur_map.get(jn, 0.0), float(joint_positions_by_name[jn]))
        self.get_logger().info(f"Ziel (deg) nach Wrap: {rad_list_to_deg([wrapped[j] for j in self.joint_order])}")

        constraints = Constraints()
        for jn in self.joint_order:
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = wrapped[jn]
            jc.tolerance_above = 1e-3
            jc.tolerance_below = 1e-3
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        mpr = MotionPlanRequest()
        mpr.group_name = self.group
        mpr.goal_constraints = [constraints]
        mpr.max_velocity_scaling_factor = float(self.vel_scale)
        mpr.max_acceleration_scaling_factor = float(self.acc_scale)
        mpr.allowed_planning_time = float(self.planning_time)
        if self.planner_id:
            mpr.planner_id = self.planner_id
        mpr.start_state = cur

        req = GetMotionPlan.Request()
        req.motion_plan_request = mpr
        res = self._timed_call("plan_kinematic_path(JointGoal)", self.cli_plan, req)

        jt = res.motion_plan_response.trajectory.joint_trajectory
        if not jt.points:
            raise RuntimeError("MoveIt lieferte keine Trajektorie (Joint-Ziel).")
        self._log_traj(jt, label="Geplante Trajektorie (Joint)")
        return jt

    def plan_to_pose_goal(self, target_pose: PoseStamped) -> JointTrajectory:
        self.get_logger().info("Planung (Pose-Ziel) gestartet ...")
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = self.group
        ik_req.ik_request.robot_state = self.current_robot_state()
        ik_req.ik_request.pose_stamped = target_pose
        ik_req.ik_request.ik_link_name = self.ee_link
        ik_req.ik_request.avoid_collisions = True

        ik_res = self._timed_call("compute_ik", self.cli_ik, ik_req)
        if ik_res.error_code.val == ik_res.error_code.SUCCESS:
            self.get_logger().info("IK erfolgreich – plane zu IK-Gelenkwinkeln.")
            joint_positions = dict(zip(ik_res.solution.joint_state.name,
                                       ik_res.solution.joint_state.position))
            return self.plan_to_joint_goal(joint_positions)

        self.get_logger().warn(
            f"IK fehlgeschlagen (code={ik_res.error_code.val}). Plane direkt mit Pose-Constraints (langsamer)."
        )

        # Fallback: Pose-Constraints (link == ee_link)
        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base
        pos_c.link_name = self.ee_link
        sphere = SolidPrimitive(); sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [max(self.pos_tol, 1e-4)]
        pos_c.constraint_region.primitives = [sphere]
        pos_c.constraint_region.primitive_poses = [Pose(
            position=target_pose.pose.position, orientation=Quaternion(w=1.0)
        )]
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.base
        ori_c.link_name = self.ee_link
        ori_c.orientation = target_pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = max(self.rot_tol, 1e-4)
        ori_c.absolute_y_axis_tolerance = max(self.rot_tol, 1e-4)
        ori_c.absolute_z_axis_tolerance = max(self.rot_tol, 1e-4)
        ori_c.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pos_c]
        constraints.orientation_constraints = [ori_c]

        mpr = MotionPlanRequest()
        mpr.group_name = self.group
        mpr.goal_constraints = [constraints]
        mpr.max_velocity_scaling_factor = float(self.vel_scale)
        mpr.max_acceleration_scaling_factor = float(self.acc_scale)
        mpr.allowed_planning_time = float(self.planning_time)
        if self.planner_id:
            mpr.planner_id = self.planner_id
        mpr.start_state = self.current_robot_state()

        req = GetMotionPlan.Request()
        req.motion_plan_request = mpr
        res = self._timed_call("plan_kinematic_path(PoseGoal)", self.cli_plan, req)

        jt = res.motion_plan_response.trajectory.joint_trajectory
        if not jt.points:
            raise RuntimeError("MoveIt lieferte keine Trajektorie (Pose-Ziel).")
        self._log_traj(jt, label="Geplante Trajektorie (Pose)")
        return jt

    def current_ee_pose(self, frame_id: Optional[str] = None) -> PoseStamped:
        req = GetPositionFK.Request()
        req.header.frame_id = frame_id or self.base
        req.fk_link_names = [self.ee_link]
        req.robot_state = self.current_robot_state()
        res = self._timed_call("compute_fk", self.cli_fk, req)
        if res.error_code.val != res.error_code.SUCCESS or not res.pose_stamped:
            raise RuntimeError(f"FK fehlgeschlagen (code={res.error_code.val})")
        return res.pose_stamped[0]


    # ---------- ROS Service Callbacks ----------
    def handle_get_current_pose(self, request, response):
        try:
            ps = self.current_ee_pose()
            p = ps.pose.position
            o = ps.pose.orientation
            response.success = True
            response.message = f"Pose: x={p.x}, y={p.y}, z={p.z}, qx={o.x}, qy={o.y}, qz={o.z}, qw={o.w}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def handle_move_to_pose(self, request, response):
        try:
            target = PoseStamped()
            target.header.frame_id = self.base
            target.pose.position = request.position
            target.pose.orientation = request.orientation
            jt = self.plan_to_pose_goal(target)
            ok = self.execute_trajectory(jt)
            response.success = ok
            response.message = "Bewegung ausgeführt" if ok else "Bewegung fehlgeschlagen"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


rclpy.init()
node = None
try:
    node = URMotionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
finally:
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()
