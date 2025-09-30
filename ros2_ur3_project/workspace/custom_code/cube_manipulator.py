#!/usr/bin/env python3
# SimpleURController: Minimaler ROS2-Node für UR-Robotersteuerung mit MoveIt
# - Fragt die aktuelle Pose ab
# - Plant und prüft Bewegung zu einer Zielpose
# - Führt die Bewegung aus, wenn möglich

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.srv import GetPlanningScene, GetMotionPlan, GetPositionIK, GetPositionFK
from moveit_msgs.msg import PlanningSceneComponents, Constraints, JointConstraint, MotionPlanRequest
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import math

# Hilfsfunktion: Wandelt Roll-Pitch-Yaw (Eulerwinkel) in Quaternion um
def rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

# Hilfsfunktion zur Umwandlung Quaternion -> RPY
def quat_to_rpy(q):
    # ROS-Standard-Konvention (xyz, w)
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class SimpleURController(Node):
    def __init__(self):
        super().__init__('simple_ur_controller')
        # MoveIt-Gruppen und Link-Namen (ggf. anpassen)
        self.group = 'ur_manipulator'      # Name der MoveIt-Gruppe
        self.ee_link = 'tool0'             # Endeffektor-Link
        self.base = 'base_link'            # Basis-Link
        self.action_name = '/scaled_joint_trajectory_controller/follow_joint_trajectory' # Action-Server für Ausführung

        # ROS2-Service-Clients für MoveIt
        self.cli_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.cli_ik = self.create_client(GetPositionIK, '/compute_ik')
        self.cli_plan = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.cli_fk = self.create_client(GetPositionFK, '/compute_fk')
        self.exec_ac = ActionClient(self, FollowJointTrajectory, self.action_name)

        # Warten auf alle Services und Action-Server (max. 10s)
        for cli in [self.cli_scene, self.cli_ik, self.cli_plan, self.cli_fk]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('Service nicht verfügbar')
        if not self.exec_ac.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('Action-Server nicht verfügbar')

    # Holt den aktuellen Roboterzustand (Joint-Werte etc.)
    def get_robot_state(self):
        req = GetPlanningScene.Request()
        req.components = PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE)
        fut = self.cli_scene.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        return res.scene.robot_state

    # Fragt die aktuelle Pose des Endeffektors ab (FK-Service)
    def get_current_pose(self):
        req = GetPositionFK.Request()
        req.header.frame_id = self.base
        req.fk_link_names = [self.ee_link]
        req.robot_state = self.get_robot_state()
        fut = self.cli_fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res.error_code.val != res.error_code.SUCCESS or not res.pose_stamped:
            raise RuntimeError('FK fehlgeschlagen')
        return res.pose_stamped[0]

    # Plant eine Bewegung zu einer Zielpose (xyz + rpy)
    # Prüft Kollisionen und gibt die geplante Trajektorie zurück
    def plan_to_pose(self, x, y, z, roll, pitch, yaw):
        # Zielpose als PoseStamped erzeugen
        pose = PoseStamped()
        pose.header.frame_id = self.base
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = rpy_to_quat(roll, pitch, yaw)

        # Inverse Kinematik für Zielpose berechnen
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
            raise RuntimeError('IK fehlgeschlagen, Kollision oder Pose unerreichbar')

        # JointConstraints für die geplanten Gelenkwinkel
        constraints = Constraints()
        for name, pos in zip(ik_res.solution.joint_state.name, ik_res.solution.joint_state.position):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 1e-3
            jc.tolerance_below = 1e-3
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        # MotionPlanRequest für MoveIt
        mpr = MotionPlanRequest()
        mpr.group_name = self.group
        mpr.goal_constraints = [constraints]
        mpr.start_state = self.get_robot_state()
        mpr.max_velocity_scaling_factor = 0.5
        mpr.max_acceleration_scaling_factor = 0.5
        mpr.allowed_planning_time = 2.0

        # Bewegungsplanung anfragen
        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = mpr
        fut = self.cli_plan.call_async(plan_req)
        rclpy.spin_until_future_complete(self, fut)
        plan_res = fut.result()
        jt = plan_res.motion_plan_response.trajectory.joint_trajectory
        if not jt.points:
            raise RuntimeError('Keine Trajektorie gefunden (Kollision?)')
        return jt

    # Führt die geplante Trajektorie aus (Action-Client)
    def execute_trajectory(self, jt: JointTrajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        fut = self.exec_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError('Goal abgelehnt')
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result()
        if result is None or getattr(result.result, 'error_code', 0) != 0:
            raise RuntimeError('Ausführung fehlgeschlagen')
        print('Bewegung erfolgreich ausgeführt!')


# Hauptfunktion: Initialisiert ROS2, fragt Pose ab, plant und führt Bewegung aus
def main():
    rclpy.init()
    node = SimpleURController()

    # Tischfläche definieren (z.B. x/y-Min/Max, z als Tischhöhe)
    table_x_min = 0.2
    table_x_max = 0.8
    table_y_min = -0.3
    table_y_max = 0.3
    table_z = 0.05  # Tischhöhe (anpassen!)
    hover_height = 0.15  # Höhe über Tisch zum Anfahren
    cube_height = 0.05   # Höhe des Würfels

    print('Bitte Startkoordinaten für den Würfel eingeben (x y):')
    start_x, start_y = map(float, input().split())
    print('Bitte Zielkoordinaten für den Würfel eingeben (x y):')
    goal_x, goal_y = map(float, input().split())

    # Orientierung: Greifer zeigt nach unten (z-Achse)
    roll = math.pi
    pitch = 0.0
    yaw = 0.0

    # 1. Über Startposition fahren (hover)
    print('Fahre über Startposition...')
    jt_hover_start = node.plan_to_pose(
        x=start_x, y=start_y, z=table_z + hover_height,
        roll=roll, pitch=pitch, yaw=yaw
    )
    node.execute_trajectory(jt_hover_start)

    # 2. Zum Greifen absenken (über dem Würfel)
    print('Senke zum Greifen ab...')
    jt_grab = node.plan_to_pose(
        x=start_x, y=start_y, z=table_z + cube_height/2,
        roll=roll, pitch=pitch, yaw=yaw
    )
    node.execute_trajectory(jt_grab)

    # Greifer schließen (Platzhalter)
    print('Greifer schließen (Platzhalter)')
    # TODO: Greifer-Logik einfügen

    # 3. Würfel anheben auf feste Ebene
    move_height = table_z + 0.05  # 5cm über Tisch
    print(f'Hebe Würfel auf {move_height:.2f}m an...')
    jt_lift = node.plan_to_pose(
        x=start_x, y=start_y, z=move_height,
        roll=roll, pitch=pitch, yaw=yaw
    )
    node.execute_trajectory(jt_lift)

    # 4. In Ebene zur Zielposition fahren
    print('Fahre in Ebene zur Zielposition...')
    jt_move_plane = node.plan_to_pose(
        x=goal_x, y=goal_y, z=move_height,
        roll=roll, pitch=pitch, yaw=yaw
    )
    node.execute_trajectory(jt_move_plane)

    # 5. Zum Absetzen absenken
    print('Senke zum Absetzen ab...')
    jt_drop = node.plan_to_pose(
        x=goal_x, y=goal_y, z=table_z + cube_height/2,
        roll=roll, pitch=pitch, yaw=yaw
    )
    node.execute_trajectory(jt_drop)

    # Greifer öffnen (Platzhalter)
    print('Greifer öffnen (Platzhalter)')
    # TODO: Greifer-Logik einfügen

    # 6. Wieder hochfahren
    print('Fahre wieder nach oben...')
    jt_lift_goal = node.plan_to_pose(
        x=goal_x, y=goal_y, z=move_height,
        roll=roll, pitch=pitch, yaw=yaw
    )
    node.execute_trajectory(jt_lift_goal)

    node.destroy_node()
    rclpy.shutdown()

# Einstiegspunkt: Startet das Programm
if __name__ == '__main__':
    main()