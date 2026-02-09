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
from std_msgs.msg import String  # <--- Added for operating_mode

# --- NEW IMPORTS FOR POSITION CHECKING ---
from tf2_ros import Buffer, TransformListener
# -----------------------------------------

import time
import threading
import copy
import math 

class PickAndPlaceTrajectory(Node):
    def __init__(self):
        super().__init__('pick_and_place_traj')
        
        # --- CLIENTS ---
        self._move_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_cli = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._ik_cli = self.create_client(GetPositionIK, 'compute_ik')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory')
        
        # --- NEW: TF LISTENER (To check real position) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # -------------------------------------------------

        # --- VISION ---
        self.subscription = self.create_subscription(Point, '/block_coordinates_tf', self.listener_callback, 10)
        # Inside each node's __init__
        self.current_mode = "" # Initialize empty
        self.mode_sub = self.create_subscription(
            String,
            'operating_mode',
            self.mode_callback,
            10
        )
        self.is_busy = False 
        
        print("--- TRAJECTORY PICK & DROP NODE STARTED ---")

        print("Waiting for Servers...")
        self._move_client.wait_for_server()
        self._execute_client.wait_for_server()
        self._cartesian_cli.wait_for_service()
        self._ik_cli.wait_for_service()
        self._gripper_client.wait_for_server()
        print("Ready! Waiting for new blocks...")

    def mode_callback(self, msg):
        self.current_mode = msg.data
        # Optional: Log the change for debugging
        # self.get_logger().info(f"Mode changed to: {self.current_mode}")

    def listener_callback(self, msg):
        if self.is_busy: return
        self.is_busy = True 
        print(f"\n>>> New Block Detected: X={msg.x:.3f}, Y={msg.y:.3f}, Z={msg.z:.3f}")
        threading.Thread(target=self.execute_mission, args=(msg,)).start()

    # --- HELPER: CHECK IF ROBOT IS REALLY THERE ---
    def wait_for_real_arrival(self, target_pose, tolerance=0.05, timeout=5.0):
        """
        Blocks execution until the robot's actual TCP is close to target_pose.
        This prevents the 'Early Drop' bug.
        """
        print(">>> Verifying arrival...")
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                # Get current position of the end effector (panda_link8)
                # Note: If your robot uses 'panda_hand', change 'panda_link8' to 'panda_hand'
                t = self.tf_buffer.lookup_transform('world', 'panda_link8', rclpy.time.Time())
                
                # Calculate distance
                dx = t.transform.translation.x - target_pose.position.x
                dy = t.transform.translation.y - target_pose.position.y
                dz = t.transform.translation.z - target_pose.position.z
                dist = math.sqrt(dx**2 + dy**2 + dz**2)
                
                if dist < tolerance:
                    return True # We are there!
            except Exception as e:
                pass # TF not ready yet, keep trying
            
            time.sleep(0.1)
        
        print(">>> Warning: Verification timed out, but proceeding.")
        return False

    def get_downward_orientation(self, yaw_degrees=0.0):
        yaw_rad = math.radians(yaw_degrees)
        q_x = math.cos(yaw_rad / 2.0)
        q_y = math.sin(yaw_rad / 2.0)
        return Quaternion(x=q_x, y=q_y, z=0.0, w=0.0)

    def control_gripper(self, action):
        print(f">>> GRIPPER: {action}...")
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        
        point = JointTrajectoryPoint()
        if action == "OPEN":
            point.positions = [0.036, 0.036] 
        else:
            point.positions = [0.00, 0.00] 
            
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
            time.sleep(0.5) 
            return True
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
        
        if response.error_code.val != 1:
            print("!!! IK Failed !!!"); return False

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
        
        # Even if "Done", we rely on the wait_for_real_arrival function now
        return True 

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

    def execute_mission(self, point):
        # ---------------------------------------------------------
        GRIPPER_ROTATION = 45.0 
        # ---------------------------------------------------------
        if self.current_mode != "cleaning":
            print(f"Received block coordinates but not in clean table mode (current mode: {self.current_mode}). Ignoring.")
            self.is_busy = False
            return
        master_orientation = self.get_downward_orientation(GRIPPER_ROTATION)
        
        approach_z = 0.35
        grasp_z = point.z + 0.10
        
        current_pose = Pose()
        current_pose.orientation = master_orientation

        print("--- MISSION START ---")
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

        # 5. TO DUSTBIN
        print("Moving to Dustbin...")
        current_pose.position.x = 0.0
        current_pose.position.y = 0.5
        current_pose.position.z = 0.5
        
        # EXECUTE MOVE
        self.move_to_pose(current_pose)
        
        # *** NEW: FORCE WAIT FOR PHYSICAL ARRIVAL ***
        # This loop will pause execution until the robot is ACTUALLY at the bin.
        # It compensates for the "False Success" signal from MoveIt.
        self.wait_for_real_arrival(current_pose)
        
        print("*** REACHED DUSTBIN ***")

        # 6. DROP
        self.control_gripper("OPEN")
        time.sleep(1.0) 
        
        # 7. FINISH & RESET
        self.move_to_home()
        print("*** MISSION COMPLETE ***")
        print(">>> Ready for new blocks...\n")
        
        self.is_busy = False 

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceTrajectory()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()