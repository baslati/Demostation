#!/usr/bin/env python3
"""
Kleines Testprogramm: Bewege TCP 5cm nach oben von aktueller Position
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState, MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import time

class TestMoveUpNode(Node):
    def __init__(self):
        super().__init__("test_move_up_node")
        
        self.group = "ur_manipulator"
        self.ee_link = "tcp"
        self.base = "base_link"
        
        self.cli_fk = self.create_client(GetPositionFK, "/compute_fk")
        self.cli_plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.exec_ac = ActionClient(self, FollowJointTrajectory, "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        
        for cli in [self.cli_fk, self.cli_plan]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("Service nicht verfügbar")
        if not self.exec_ac.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Action Server nicht verfügbar")
        
        self.get_logger().info("Services bereit")
        
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        
        # Warte auf joint states
        self.get_logger().info("Warte auf /joint_states...")
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Joint states erhalten")
        
        self.test_move_up()
    
    def _wait_future(self, fut, label, timeout_sec=20.0):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.01)
        if not fut.done():
            raise RuntimeError(f"Timeout bei {label}")
        exc = fut.exception()
        if exc:
            raise RuntimeError(f"{label} Exception: {exc}")
        return fut.result()

    def _on_js(self, msg):
        self.js = msg
    
    def get_current_pose(self):
        req = GetPositionFK.Request()
        req.header.frame_id = self.base
        req.fk_link_names = [self.ee_link]
        req.robot_state.joint_state = self.js
        
        self.get_logger().info("[DEBUG] FK Request gesendet, warte auf Antwort...")
        self.get_logger().info(f"[DEBUG] JointState: {self.js}")
        
        fut = self.cli_fk.call_async(req)
        res = self._wait_future(fut, "FK")
        if res is None or not res.pose_stamped:
            raise RuntimeError("FK fehlgeschlagen")
        self.get_logger().info(f"[DEBUG] FK Response: {res}")
        return res.pose_stamped[0]
    
    def plan_to_pose(self, pose):
        constraints = Constraints()
        
        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base
        pos_c.link_name = self.ee_link
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
        pos_c.constraint_region.primitives = [sphere]
        pos_c.constraint_region.primitive_poses = [PoseStamped(pose=pose.pose)]
        pos_c.weight = 1.0
        
        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.base
        ori_c.link_name = self.ee_link
        ori_c.orientation = pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.2
        ori_c.absolute_y_axis_tolerance = 0.2
        ori_c.absolute_z_axis_tolerance = 0.3
        ori_c.weight = 1.0
        
        constraints.position_constraints = [pos_c]
        constraints.orientation_constraints = [ori_c]
        
        mpr = MotionPlanRequest()
        mpr.group_name = self.group
        mpr.goal_constraints = [constraints]
        mpr.start_state.joint_state = self.js
        mpr.allowed_planning_time = 15.0
        mpr.num_planning_attempts = 10
        
        plan_req = GetMotionPlan.Request(motion_plan_request=mpr)
        fut = self.cli_plan.call_async(plan_req)
        res = self._wait_future(fut, "Planning")
        if res is None or not res.motion_plan_response.trajectory.joint_trajectory.points:
            raise RuntimeError("Planning fehlgeschlagen")
        return res.motion_plan_response.trajectory.joint_trajectory
    
    def execute(self, jt):
        goal = FollowJointTrajectory.Goal(trajectory=jt)
        fut = self.exec_ac.send_goal_async(goal)
        gh = self._wait_future(fut, "SendGoal")
        if not gh.accepted:
            raise RuntimeError("Goal rejected")
        res_fut = gh.get_result_async()
        result = self._wait_future(res_fut, "Execute")
        if result.result.error_code != 0:
            raise RuntimeError("Execution failed")
    
    def test_move_up(self):
        try:
            current_pose = self.get_current_pose()
            self.get_logger().info(f"Aktuelle Pose: x={current_pose.pose.position.x:.3f}, y={current_pose.pose.position.y:.3f}, z={current_pose.pose.position.z:.3f}")
            
            # 5cm nach oben
            new_pose = PoseStamped()
            new_pose.header = current_pose.header
            new_pose.pose = current_pose.pose
            new_pose.pose.position.z += 0.05
            
            self.get_logger().info(f"Ziel Pose: x={new_pose.pose.position.x:.3f}, y={new_pose.pose.position.y:.3f}, z={new_pose.pose.position.z:.3f}")
            
            jt = self.plan_to_pose(new_pose)
            self.get_logger().info("Planning erfolgreich")
            
            self.execute(jt)
            self.get_logger().info("Bewegung erfolgreich")
            
        except Exception as e:
            self.get_logger().error(f"Fehler: {e}")

def main():
    rclpy.init()
    node = TestMoveUpNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()