import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import time
import threading
import copy

class PickAndPlaceTrajectory(Node):
    def __init__(self):
        super().__init__('pick_and_place_traj')
        
        # --- CLIENTS ---
        self._move_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_cli = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._ik_cli = self.create_client(GetPositionIK, 'compute_ik')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory')
        
        # --- VISION ---
        self.subscription = self.create_subscription(Point, '/block_coordinates_tf', self.listener_callback, 10)
        self.target_received = False
        
        print("--- TRAJECTORY PICK & DROP NODE STARTED ---")
        print("Waiting for Servers...")
        self._move_client.wait_for_server()
        self._execute_client.wait_for_server()
        self._cartesian_cli.wait_for_service()
        self._ik_cli.wait_for_service()
        self._gripper_client.wait_for_server()
        print("Ready! Waiting for Vision...")

    def listener_callback(self, msg):
        if self.target_received: return
        self.target_received = True 
        print(f"Target Acquired: X={msg.x:.3f}, Y={msg.y:.3f}, Z={msg.z:.3f}")
        threading.Thread(target=self.execute_mission, args=(msg,)).start()

    # --- HELPER: FIXED ORIENTATION ---
    def get_downward_orientation(self):
        # Keeps the gripper pointing down (Panda standard: x=1 means 180 deg rotation on X)
        return Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    # --- HELPER: ROBUST GRIPPER CONTROL ---
    def control_gripper(self, action):
        """
        Sends command and WAITS for completion.
        Returns True only if the gripper reports success.
        """
        print(f">>> GRIPPER: {action}...")
        
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        
        point = JointTrajectoryPoint()
        if action == "OPEN":
            point.positions = [0.036, 0.036] # Max width
        else:
            point.positions = [0.00, 0.00] # Closed
            
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points.append(point)
        goal.trajectory = traj
        
        # 1. Send Goal
        future = self._gripper_client.send_goal_async(goal)
        while not future.done(): time.sleep(0.01)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            print("!!! GRIPPER ERROR: Goal Rejected !!!")
            return False

        # 2. WAIT FOR COMPLETION (The "Proof" Mechanism)
        # This ensures we don't move the arm while fingers are still moving
        res_future = goal_handle.get_result_async()
        while not res_future.done(): time.sleep(0.01)
        
        result = res_future.result().result
        
        # Check success (ErrorCode 0 is usually success for controllers)
        if result.error_code == 0: 
            # Extra delay to ensure 'grip' is solid before lifting
            time.sleep(0.5) 
            return True
        else:
            print(f"!!! GRIPPER FAILED: Error Code {result.error_code} !!!")
            return False

    # --- ACTION: MOVE TO POSE (FANUC STYLE) ---
    def move_to_pose(self, target_pose):
        print(f">>> Moving to Pose: X={target_pose.position.x:.2f}, Z={target_pose.position.z:.2f}")

        # 1. IK Calculation
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "panda_arm"
        ik_req.ik_request.pose_stamped.header.frame_id = "world"
        ik_req.ik_request.pose_stamped.pose = target_pose
        ik_req.ik_request.avoid_collisions = True
        
        future_ik = self._ik_cli.call_async(ik_req)
        while not future_ik.done(): time.sleep(0.01)
        response = future_ik.result()
        
        if response.error_code.val != 1:
            print("!!! IK Failed: Target Unreachable !!!")
            return False

        # 2. Extract Joints
        target_joints = response.solution.joint_state.position
        target_names = response.solution.joint_state.name
        
        arm_joints = []
        arm_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        for name in arm_names:
            try:
                arm_joints.append(target_joints[target_names.index(name)])
            except: return False

        # 3. Move Joints
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "panda_arm"
        constraints = Constraints()
        for i in range(7):
            jc = JointConstraint()
            jc.joint_name = arm_names[i]
            jc.position = arm_joints[i]
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)
        
        future = self._move_client.send_goal_async(goal_msg)
        while not future.done(): time.sleep(0.01)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Move Rejected!")
            return False

        # 4. Wait for Result
        res_future = goal_handle.get_result_async()
        while not res_future.done(): time.sleep(0.01)
        
        # Add a small settlement delay
        time.sleep(0.2)
        # return res_future.result().result.error_code.val == 1
        return True
    
    # --- ACTION: CARTESIAN SEQUENCE ---
    def execute_cartesian_move(self, start_pose, end_z):
        waypoints = []
        w1 = copy.deepcopy(start_pose)
        waypoints.append(w1)
        w2 = copy.deepcopy(w1)
        w2.position.z = end_z
        waypoints.append(w2)

        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = "panda_arm"
        req.waypoints = waypoints
        req.max_step = 0.01; req.jump_threshold = 0.0; req.avoid_collisions = True
        
        future = self._cartesian_cli.call_async(req)
        while not future.done(): time.sleep(0.01)
        response = future.result()
        
        if response.fraction < 0.90: 
            print("Cartesian Path incomplete"); return False
            
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        
        future_exec = self._execute_client.send_goal_async(goal)
        while not future_exec.done(): time.sleep(0.01)
        
        goal_handle = future_exec.result()
        if not goal_handle.accepted:
            print("Trajectory Rejected!")
            return False

        res_future = goal_handle.get_result_async()
        while not res_future.done(): time.sleep(0.01)
        
        # Add settlement delay
        time.sleep(0.2)
        return res_future.result().result.error_code.val == 1
    
    def move_to_home(self):
        home_pose = Pose()
        home_pose.position.x = 0.3; home_pose.position.y = 0.0; home_pose.position.z = 0.5
        home_pose.orientation = self.get_downward_orientation()
        return self.move_to_pose(home_pose)

    # --- MAIN MISSION LOGIC ---
    def execute_mission(self, point):
        master_orientation = self.get_downward_orientation()
        
        approach_z = 0.35
        grasp_z = point.z + 0.10
        
        current_pose = Pose()
        current_pose.orientation = master_orientation

        print("--- MISSION START ---")
        self.control_gripper("OPEN")
        # Delay to ensure full open before moving
        time.sleep(0.5)

        # 1. APPROACH
        current_pose.position.x = point.x
        current_pose.position.y = point.y
        current_pose.position.z = approach_z
        if not self.move_to_pose(current_pose): return
        time.sleep(0.5) # Stable check

        # 2. DOWN
        if not self.execute_cartesian_move(current_pose, grasp_z): return
        time.sleep(0.5) # Ensure stopped completely

        # 3. GRAB
        print(">>> GRASPING...")
        if not self.control_gripper("CLOSE"): return
        # The control_gripper function now has internal delays too

        # 4. UP
        grasp_pose = copy.deepcopy(current_pose)
        grasp_pose.position.z = grasp_z
        if not self.execute_cartesian_move(grasp_pose, approach_z): return

        # 5. TO DUSTBIN
        print("Moving to Dustbin...")
        current_pose.position.x = 0.0
        current_pose.position.y = 0.5
        current_pose.position.z = 0.5
        if not self.move_to_pose(current_pose): return
        time.sleep(0.5)
        print("*** DONE to dustbin ***")


        # 6. DROP
        self.control_gripper("OPEN")
        time.sleep(1.0) # Ensure object falls out
        
        # 7. FINISH
        self.move_to_home()
        print("*** DONE ***")

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceTrajectory()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()