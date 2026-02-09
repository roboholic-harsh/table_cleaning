#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
import time
import threading
import copy
import math 
from std_msgs.msg import String  # <--- Added for operating_mode


class AutomaticSorter(Node):
    def __init__(self):
        super().__init__('automatic_sorter')
        
        # --- CLIENTS (Untouched) ---
        self._move_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_cli = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._ik_cli = self.create_client(GetPositionIK, 'compute_ik')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory')
        
        # --- TF (Untouched) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- MEMORY FOR SORTER ---
        self.latest_red = None
        self.latest_blue = None
        self.latest_green = None
        self.is_busy = False 
        
        # --- NEW SUBSCRIPTIONS ---
        self.create_subscription(Point, '/red_block_coords', self.red_callback, 10)
        self.create_subscription(Point, '/blue_block_coords', self.blue_callback, 10)
        self.create_subscription(Point, '/green_block_coords', self.green_callback, 10)
        # Inside each node's __init__
        self.current_mode = "" # Initialize empty
        self.mode_sub = self.create_subscription(
            String,
            'operating_mode',
            self.mode_callback,
            10
        )
        # --- BIN LOCATIONS ---
        # Adjust 'y' or 'x' slightly if bins are placed differently
        self.bin_blue   = {'x': 0.0, 'y': -0.5, 'z': 0.5}  # Right
        self.bin_red  = {'x': 0.0, 'y': 0.5, 'z': 0.5}   # Left
        self.bin_green = {'x': -0.5, 'y': 0.0, 'z': 0.5}  # Back

        print("--- AUTOMATIC SORTER STARTED ---")
        print("Waiting for Servers...")
        self._move_client.wait_for_server()
        self._execute_client.wait_for_server()
        self._cartesian_cli.wait_for_service()
        self._ik_cli.wait_for_service()
        self._gripper_client.wait_for_server()
        
        print("Ready! Starting Priority Sorting Loop...")
        threading.Thread(target=self.sorting_brain_loop).start()

    # --- CALLBACKS ---
    def red_callback(self, msg): self.latest_red = msg
    def blue_callback(self, msg): self.latest_blue = msg
    def green_callback(self, msg): self.latest_green = msg
    def mode_callback(self, msg):
        self.current_mode = msg.data
        # Optional: Log the change for debugging
        # self.get_logger().info(f"Mode changed to: {self.current_mode}")
    # --- BRAIN LOOP (Priority Logic) ---

    def sorting_brain_loop(self):
        while rclpy.ok():
            if not self.is_busy:
                # 1. RED (Highest Priority)
                if self.latest_red is not None:
                    target = self.latest_red
                    self.latest_red = None 
                    self.execute_mission(target, "RED")
                
                # 2. BLUE
                elif self.latest_blue is not None:
                    target = self.latest_blue
                    self.latest_blue = None
                    self.execute_mission(target, "BLUE")

                # 3. GREEN
                elif self.latest_green is not None:
                    target = self.latest_green
                    self.latest_green = None
                    self.execute_mission(target, "GREEN")
                
                else:
                    time.sleep(0.5)
            time.sleep(0.1)

    # =========================================================================
    #  MOVEMENT FUNCTIONS (STRICTLY PRESERVED FROM YOUR CODE)
    # =========================================================================

    def wait_for_real_arrival(self, target_pose, tolerance=0.05, timeout=5.0):
        print(">>> Verifying arrival...")
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                t = self.tf_buffer.lookup_transform('world', 'panda_link8', rclpy.time.Time())
                dx = t.transform.translation.x - target_pose.position.x
                dy = t.transform.translation.y - target_pose.position.y
                dz = t.transform.translation.z - target_pose.position.z
                dist = math.sqrt(dx**2 + dy**2 + dz**2)
                if dist < tolerance: return True 
            except Exception as e: pass 
            time.sleep(0.1)
        print(">>> Warning: Verification timed out, but proceeding.")
        return False

    def get_downward_orientation(self, yaw_degrees=0.0):
        yaw_rad = math.radians(yaw_degrees)
        q_x = math.cos(yaw_rad / 2.0); q_y = math.sin(yaw_rad / 2.0)
        return Quaternion(x=q_x, y=q_y, z=0.0, w=0.0)

    def control_gripper(self, action):
        print(f">>> GRIPPER: {action}...")
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        point = JointTrajectoryPoint()
        if action == "OPEN": point.positions = [0.036, 0.036] 
        else: point.positions = [0.00, 0.00] 
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points.append(point)
        goal.trajectory = traj
        future = self._gripper_client.send_goal_async(goal)
        while not future.done(): time.sleep(0.01)
        goal_handle = future.result()
        if not goal_handle.accepted: return False
        res_future = goal_handle.get_result_async()
        while not res_future.done(): time.sleep(0.01)
        if res_future.result().result.error_code == 0: 
            time.sleep(0.5); return True
        else: return False 

    def move_to_pose(self, target_pose):
        print(f">>> Moving to Pose: X={target_pose.position.x:.2f}, Z={target_pose.position.z:.2f}")
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "panda_arm"
        ik_req.ik_request.pose_stamped.header.frame_id = "world"
        ik_req.ik_request.pose_stamped.pose = target_pose
        ik_req.ik_request.avoid_collisions = True
        future_ik = self._ik_cli.call_async(ik_req)
        while not future_ik.done(): time.sleep(0.01)
        response = future_ik.result()
        if response.error_code.val != 1: print("!!! IK Failed !!!"); return False

        target_joints = response.solution.joint_state.position
        target_names = response.solution.joint_state.name
        arm_joints = []
        arm_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        for name in arm_names:
            try: arm_joints.append(target_joints[target_names.index(name)])
            except: return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "panda_arm"
        constraints = Constraints()
        for i in range(7):
            jc = JointConstraint()
            jc.joint_name = arm_names[i]; jc.position = arm_joints[i]
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)
        future = self._move_client.send_goal_async(goal_msg)
        while not future.done(): time.sleep(0.01)
        goal_handle = future.result()
        if not goal_handle.accepted: return False
        res_future = goal_handle.get_result_async()
        while not res_future.done(): time.sleep(0.01)
        return True 

    def execute_cartesian_move(self, start_pose, end_z):
        waypoints = []
        w1 = copy.deepcopy(start_pose)
        waypoints.append(w1)
        w2 = copy.deepcopy(w1); w2.position.z = end_z
        waypoints.append(w2)
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"; req.group_name = "panda_arm"; req.waypoints = waypoints
        req.max_step = 0.01; req.jump_threshold = 0.0; req.avoid_collisions = True
        future = self._cartesian_cli.call_async(req)
        while not future.done(): time.sleep(0.01)
        response = future.result()
        if response.fraction < 0.90: return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        future_exec = self._execute_client.send_goal_async(goal)
        while not future_exec.done(): time.sleep(0.01)
        goal_handle = future_exec.result()
        if not goal_handle.accepted: return False
        res_future = goal_handle.get_result_async()
        while not res_future.done(): time.sleep(0.01)
        time.sleep(0.2)
        return True

    def move_to_home(self):
        home_pose = Pose()
        home_pose.position.x = 0.3; home_pose.position.y = 0.0; home_pose.position.z = 0.5
        home_pose.orientation = self.get_downward_orientation(0)
        return self.move_to_pose(home_pose)

    # =========================================================================
    #  EXECUTE MISSION (Logic Updated for Sorting)
    # =========================================================================

    def execute_mission(self, point, color_type):
        if self.current_mode != "sorting":
            print(f"Received block coordinates but not in joint_control mode (current mode: {self.current_mode}). Ignoring.")
            self.is_busy = False
            return
        self.is_busy = True
        GRIPPER_ROTATION = 45.0 
        
        master_orientation = self.get_downward_orientation(GRIPPER_ROTATION)
        approach_z = 0.35
        grasp_z = point.z + 0.10
        
        current_pose = Pose()
        current_pose.orientation = master_orientation

        print(f"--- STARTING PICK ({color_type}) ---")
        self.control_gripper("OPEN")
        time.sleep(0.5)

        # 1. APPROACH
        current_pose.position.x = point.x
        current_pose.position.y = point.y
        current_pose.position.z = approach_z
        if not self.move_to_pose(current_pose): 
            print("Approach Failed."); self.is_busy = False; return
        time.sleep(0.5) 

        # 2. DOWN
        if not self.execute_cartesian_move(current_pose, grasp_z): 
            print("Down Failed."); self.is_busy = False; return
        time.sleep(0.5) 

        # 3. GRAB
        print(">>> GRASPING...")
        if not self.control_gripper("CLOSE"): 
            print("Grip Failed."); self.is_busy = False; return

        # 4. UP
        grasp_pose = copy.deepcopy(current_pose)
        grasp_pose.position.z = grasp_z
        if not self.execute_cartesian_move(grasp_pose, approach_z): 
            print("Up Failed."); self.is_busy = False; return

        # 5. TO BIN (Depends on Color)
        print(f"Moving to {color_type} Bin...")
        bin_coords = None
        if color_type == "RED": bin_coords = self.bin_red
        elif color_type == "BLUE": bin_coords = self.bin_blue
        elif color_type == "GREEN": 
            bin_coords = self.bin_blue
            current_pose.position.x = bin_coords['x']
            current_pose.position.y = bin_coords['y']
            current_pose.position.z = bin_coords['z']
            self.move_to_pose(current_pose)
            self.wait_for_real_arrival(current_pose)

            bin_coords = self.bin_green
        
        current_pose.position.x = bin_coords['x']
        current_pose.position.y = bin_coords['y']
        current_pose.position.z = bin_coords['z']
        
        self.move_to_pose(current_pose)
        
        # *** PHYSICAL CHECK (Required to fix early release) ***
        self.wait_for_real_arrival(current_pose)
        
        print(f"*** REACHED {color_type} BIN ***")

        # 6. DROP
        self.control_gripper("OPEN")
        time.sleep(1.0) 
        
        if color_type == "GREEN": 
            bin_coords = self.bin_blue
            current_pose.position.x = bin_coords['x']
            current_pose.position.y = bin_coords['y']
            current_pose.position.z = bin_coords['z'] 
            self.move_to_pose(current_pose)
            self.wait_for_real_arrival(current_pose)
        
        # 7. HOME
        self.move_to_home()
        print("--- MISSION COMPLETE ---\n")
        
        self.is_busy = False 

def main(args=None):
    rclpy.init(args=args)
    node = AutomaticSorter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()