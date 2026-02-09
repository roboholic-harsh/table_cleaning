#!/usr/bin/env python3

import json
import math
import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from std_msgs.msg import String  # <--- Added for operating_mode

from custom_interfaces.action import ArmCommand
from sensor_msgs.msg import JointState #########################
from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions
)


class ArmActionServer(Node):

    def __init__(self):
        super().__init__("arm_action_server")

        self.get_logger().info("Starting Arm Action Server (MoveIt API)")

        # -------------------------------------------------
        # MoveIt MoveGroup Action Client (already running)
        # -------------------------------------------------
        self.movegroup_client = ActionClient(
            self,
            MoveGroup,
            "/move_action"
        )
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory')

        # -------------------------------------------------
        # NLP → Robot Action Server
        # -------------------------------------------------
        self.action_server = ActionServer(
            self,
            ArmCommand,
            "arm_command",
            self.execute_callback
        )
        # Inside each node's __init__
        self.current_mode = "" # Initialize empty
        self.mode_sub = self.create_subscription(
            String,
            'operating_mode',
            self.mode_callback,
            10
        )

        self.get_logger().info("Arm Action Server ready")
        self.current_joint_state = None 
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10
            )
        
    def mode_callback(self, msg):
        self.current_mode = msg.data
        # Optional: Log the change for debugging
        # self.get_logger().info(f"Mode changed to: {self.current_mode}")

    def joint_state_cb(self, msg):
        self.current_joint_state = msg
    
    def control_gripper(self, action):
        self.get_logger().info(f">>> GRIPPER: {action}...")
        
        # 1. Create the Goal
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

        # 2. Send the Goal and spin until it's sent
        send_future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        # 3. Wait for the Result and spin until it's finished
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().result
        if status.error_code == 0:
            time.sleep(0.5) # Allow physical settling
            return True
        else:
            self.get_logger().error(f"Gripper failed with code: {status.error_code}")
            return False
    # ==================================================
    # ACTION CALLBACK
    # ==================================================
    def execute_callback(self, goal_handle):
        if self.current_mode != "joint_control":
            self.get_logger().info("Received ArmCommand goal but not in joint_control mode")
            goal_handle.abort()
            return ArmCommand.Result(
                success=False,
                message="in " + self.current_mode + " mode, cannot execute joint command"
            )
        self.get_logger().info("Received ArmCommand goal as : " + goal_handle.request.json_command)

        # -------------------------------------------------
        # 1. Parse JSON
        # -------------------------------------------------
        try:
            cmd = json.loads(goal_handle.request.json_command)
        except Exception as e:
            goal_handle.abort()
            return ArmCommand.Result(success=False, message=str(e))

        if "move" not in cmd or "gripper" not in cmd:
            self.get_logger().info("Received ArmCommand goal:JSON must contain 'move' and 'gripper'")

            goal_handle.abort()
            return ArmCommand.Result(
                success=False,
                message="JSON must contain 'move' and 'gripper'"

            )

        # -------------------------------------------------
        # 2. ARM COMMAND
        # -------------------------------------------------
        arm_success = self.send_joint_goal(
            group_name="panda_arm",
            joint_names=[
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ],
            joint_values=cmd["move"]
        )

        if not arm_success:
            goal_handle.abort()
            return ArmCommand.Result(
                success=False,
                message="Arm execution failed - while initilizing"
            )

        # -------------------------------------------------
        # 3. GRIPPER COMMAND
        # -------------------------------------------------
        if cmd["gripper"][0] == 1:
            self.control_gripper("CLOSE")
        elif cmd["gripper"][0] == 0:
            self.control_gripper("OPEN")
        else:
            print('no gripper command or invalid command received, skipping gripper control')

        # -------------------------------------------------
        # 4. SUCCESS
        # -------------------------------------------------
        goal_handle.succeed()
        return ArmCommand.Result(
            success=True,
            message="Arm and gripper executed successfully"
        )

    # ==================================================
    # MOVEGROUP GOAL SENDER
    # ==================================================
    def send_joint_goal(self, group_name, joint_names, joint_values, radians=False):

        self.get_logger().info(f"Sending MoveGroup goal for {group_name}")
##############################################################################
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received yet")
            return False
##############################################################################


        # Wait for move_group
        if not self.movegroup_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available")
            return False
#######################################################
        current_positions = dict(
            zip(self.current_joint_state.name,
                self.current_joint_state.position)
        )
#######################################################


        constraints = Constraints()

        for name, value in zip(joint_names, joint_values):
            # if value == -1:
            #     continue

            jc = JointConstraint()
            jc.joint_name = name

            # jc.position = value if radians else math.radians(value)

#####################################
            if value == -402:
                # 🔒 LOCK joint at current position
                jc.position = current_positions[name]
            else:
                jc.position = value if radians else math.radians(value)
#####################################

            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name = group_name
        request.goal_constraints.append(constraints)
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0

        goal = MoveGroup.Goal()
        goal.request = request

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        send_goal_future = self.movegroup_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        return result.error_code.val == result.error_code.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = ArmActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
