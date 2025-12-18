import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import (
    PositionIKRequest,
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    RobotState
)
from moveit_msgs.action import ExecuteTrajectory

from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import GripperCommand

import math
from geometry_msgs.msg import Quaternion

def make_quaternion_from_euler(roll, pitch, yaw):
    q = Quaternion()
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q


# “Straight” gripper: keep default panda_link8 orientation (identity)
TOP_DOWN_Q = Quaternion()
TOP_DOWN_Q.x = 0.0
TOP_DOWN_Q.y = 0.0
TOP_DOWN_Q.z = 0.0
TOP_DOWN_Q.w = 1.0




class PickPlaceNode(Node):
    def __init__(self) -> None:
        

        super().__init__("pick_place_node")
        self.get_logger().info("Pick & Place node started. Waiting for YOLO pose...")

        # Latest data
        self.current_joint_state: JointState | None = None
        self.latest_object_pose: PoseStamped | None = None
        self.pipeline_started: bool = False

        # Subscribers
        self.create_subscription(
            PoseStamped,
            "/detected_object_pose",
            self.pose_callback,
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )
        
        # for gripper
        self.gripper_client = ActionClient(self,GripperCommand,'/panda_hand_controller/gripper_cmd')
        
        # Clients
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

        # Timer that will trigger the pipeline once everything is ready
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.logged_wait_ik = False
        self.logged_wait_plan = False
        self.logged_wait_exec = False
        self.logged_wait_joint = False
        self.logged_wait_pose = False
        
        # Action client for executing trajectories
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # Make sure we don't start multiple pipelines in parallel
        self.busy = False

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def joint_state_callback(self, msg: JointState) -> None:
        self.current_joint_state = msg

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_object_pose = msg
        self.get_logger().info(
            f"Received object pose: x={msg.pose.position.x:.3f}, "
            f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
        )

    def timer_callback(self):
        """Check if prerequisites are ready, then launch pipeline in a background thread."""
        if self.pipeline_started:
            return

        if self.current_joint_state is None:
            return

        if self.latest_object_pose is None:
            return

        if not self.ik_client.service_is_ready():
            return

        if not self.plan_client.service_is_ready():
            return

        if not self.execute_client.wait_for_server(timeout_sec=0.1):
            return

        # All ready, start pipeline ONCE
        self.pipeline_started = True

        # Run pipeline in background thread, not callback thread
        import threading
        threading.Thread(target=self.run_pick_place, daemon=True).start()

    # ------------------------------------------------------------------
    # Core pipeline
    # ------------------------------------------------------------------
    def run_pick_place(self):
        # prevent overlapping runs
        if self.busy:
            self.get_logger().warn("Pick & place already running, ignoring new detection.")
            return

        if self.latest_object_pose is None:
            self.get_logger().error("No object pose available, cannot run pick & place.")
            return

        self.busy = True

        try:
            obj = self.latest_object_pose
            self.get_logger().info(f"Starting pick & place pipeline for object at "
                                   f"x={obj.pose.position.x:.3f}, "
                                   f"y={obj.pose.position.y:.3f}, "
                                   f"z={obj.pose.position.z:.3f}")

            # 1) HOVER POSE above object (top-down approach)
            hover_pose = PoseStamped()
            hover_pose.header.frame_id = "world"
            hover_pose.pose.position.x = obj.pose.position.x
            hover_pose.pose.position.y = obj.pose.position.y
            hover_pose.pose.position.z = obj.pose.position.z + 0.25  # 10 cm above
            hover_pose.pose.orientation = TOP_DOWN_Q

            self.get_logger().info("Computing IK for HOVER pose...")
            hover_names, hover_positions = self.compute_ik(hover_pose)
            if hover_names is None:
                self.get_logger().error("IK failed for hover pose, aborting.")
                self.busy = False
                return

            hover_plan = self.plan_to_joint_goal(hover_names, hover_positions)
            if hover_plan is None or hover_plan.motion_plan_response.error_code.val != 1:
                self.get_logger().error("Planning failed for hover pose, aborting.")
                self.busy = False
                return
            
            self.execute_plan(hover_plan, "hover above object")

            # 2) GRASP POSE at object (still top-down, just lower z)
            grasp_pose = PoseStamped()
            grasp_pose.header.frame_id = "world"
            grasp_pose.pose.position.x = obj.pose.position.x
            grasp_pose.pose.position.y = obj.pose.position.y
            grasp_pose.pose.position.z = obj.pose.position.z + 0.05 # same as YOLO pose
            grasp_pose.pose.orientation = TOP_DOWN_Q


            self.get_logger().info("Computing IK for GRASP pose...")
            grasp_names, grasp_positions = self.compute_ik(grasp_pose)
            if grasp_names is None:
                self.get_logger().error("IK failed for grasp pose, aborting.")
                return

            grasp_plan = self.plan_to_joint_goal(grasp_names, grasp_positions)
            if grasp_plan is None or grasp_plan.motion_plan_response.error_code.val != 1:
                self.get_logger().error("Planning failed for grasp pose, aborting.")
                return
            
            self.execute_plan(grasp_plan, "move down to grasp")

            # close the gripper
            self.close_gripper()

            # 3) LIFT back to HOVER (reuse same hover joint goal)
            self.get_logger().info("Planning lift back to hover pose...")
            lift_plan = self.plan_to_joint_goal(hover_names, hover_positions)
            if lift_plan is None or lift_plan.motion_plan_response.error_code.val != 1:
                self.get_logger().error("Planning failed for lift-back-to-hover, aborting.")
                return
            self.execute_plan(lift_plan, "lift object")

            # 4) PLACE POSE at bin (Option A: x=0.5, y=-0.3, z=0.5)
            # 4a) high hover above bin
            place_hover_pose = PoseStamped()
            place_hover_pose.header.frame_id = "world"
            place_hover_pose.pose.position.x = 0.5
            place_hover_pose.pose.position.y = -0.3
            place_hover_pose.pose.position.z = 0.70   # high above both shelf & bin
            place_hover_pose.pose.orientation = TOP_DOWN_Q

            self.get_logger().info("Computing IK for PLACE HOVER pose (over bin)...")
            ph_names, ph_positions = self.compute_ik(place_hover_pose)
            if ph_names is None:
                self.get_logger().error("IK failed for place-hover pose, aborting.")
                return

            ph_plan = self.plan_to_joint_goal(ph_names, ph_positions)
            if ph_plan is None or ph_plan.motion_plan_response.error_code.val != 1:
                self.get_logger().error("Planning failed for place-hover pose, aborting.")
                return

            self.execute_plan(ph_plan, "move to bin hover")

            # 4b) actual place pose (above bin top, but not crazy high)
            place_pose = PoseStamped()
            place_pose.header.frame_id = "world"
            place_pose.pose.position.x = 0.5
            place_pose.pose.position.y = -0.3
            place_pose.pose.position.z = 0.40   # a bit above bin top (0.30)
            place_pose.pose.orientation = TOP_DOWN_Q

            self.get_logger().info("Computing IK for PLACE pose (bin)...")
            place_names, place_positions = self.compute_ik(place_pose)
            if place_names is None:
                self.get_logger().error("IK failed for place pose, aborting.")
                return

            place_plan = self.plan_to_joint_goal(place_names, place_positions)
            if place_plan is None or place_plan.motion_plan_response.error_code.val != 1:
                self.get_logger().error("Planning failed for place pose, aborting.")
                return

            self.execute_plan(place_plan, "move down to place")
            # open the gripper 
            self.open_gripper()

            self.get_logger().info("Pick & place pipeline completed (plans computed for all steps).")

        finally:
            self.busy = False


    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def compute_ik(self, pose_stamped):
        """Returns (joint_names, joint_positions) or (None, None)."""

        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.avoid_collisions = False
        req.ik_request.pose_stamped = pose_stamped

        self.get_logger().info("Calling /compute_ik...")
        future = self.ik_client.call_async(req)

        # Wait up to 5 seconds
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error("IK future did not complete in time.")
            return None, None

        resp = future.result()
        self.get_logger().info(f"IK result code: {resp.error_code.val}")

        if resp.error_code.val != 1:
            self.get_logger().error("IK failed.")
            return None, None

        js = resp.solution.joint_state

        self.get_logger().info(f"IK returned {len(js.name)} joints: {js.name}")

        return js.name, js.position


    def plan_to_joint_goal(self, joint_names, joint_positions):
        """
        Call /plan_kinematic_path to plan from the current robot state
        to the IK solution given by (joint_names, joint_positions).
        We deliberately do NOT set start_state and let MoveIt use the
        current state from the planning scene.
        """
        self.get_logger().info("Waiting for /plan_kinematic_path service...")
        while not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/plan_kinematic_path still not available, waiting...")
        self.get_logger().info("/plan_kinematic_path is ready!")

        # Build the request
        req = GetMotionPlan.Request()
        mreq = MotionPlanRequest()
        mreq.group_name = "panda_arm"
        mreq.num_planning_attempts = 20
        mreq.allowed_planning_time = 10.0
        mreq.max_velocity_scaling_factor = 0.2
        mreq.max_acceleration_scaling_factor = 0.2

        # ---- DO NOT TOUCH start_state ----
        # Leave mreq.start_state as default (empty), so MoveIt uses the
        # current state from the PlanningSceneMonitor.
        # ----------------------------------

        # Build joint goal constraints from IK solution
        constraints = Constraints()
        arm_joint_count = 0
        for name, pos in zip(joint_names, joint_positions):
            # Only arm joints, skip fingers
            if not name.startswith("panda_joint"):
                continue
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            arm_joint_count += 1

        if arm_joint_count == 0:
            self.get_logger().error("No arm joints found in IK solution; cannot build goal constraints.")
            return None

        mreq.goal_constraints = [constraints]

        req.motion_plan_request = mreq

        self.get_logger().info("Calling /plan_kinematic_path...")
        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            self.get_logger().error("/plan_kinematic_path future did not complete in time.")
            return None

        resp = future.result()
        if resp is None:
            self.get_logger().error("/plan_kinematic_path returned None result.")
            return None

        code = resp.motion_plan_response.error_code.val
        self.get_logger().info(f"Planning result code: {code}")

        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Planning failed with non-success error code.")
            return None

        return resp

    def execute_trajectory(self, plan_resp) -> bool:
        """Send the planned trajectory to /execute_trajectory action and wait for result."""
        traj = plan_resp.motion_plan_response.trajectory

        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory action server not available.")
            return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = traj

        self.get_logger().info("Sending trajectory to /execute_trajectory...")
        send_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done():
            self.get_logger().error("Send goal future did not complete in time.")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("ExecuteTrajectory goal was rejected.")
            return False

        self.get_logger().info("Trajectory goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        if not result_future.done():
            self.get_logger().error("ExecuteTrajectory result future did not complete in time.")
            return False

        result = result_future.result().result
        code = result.error_code.val
        self.get_logger().info(f"ExecuteTrajectory result code: {code}")
        return code == MoveItErrorCodes.SUCCESS

    def execute_plan(self, plan_resp, label=""):
        """
        Send the planned trajectory to MoveIt /execute_trajectory to actually move the arm.
        plan_resp is the response from /plan_kinematic_path (GetMotionPlan).
        """
        if plan_resp is None:
            self.get_logger().error(f"execute_plan called with None plan for {label}")
            return False

        # Wait for the ExecuteTrajectory action server
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory action server not available.")
            return False

        goal = ExecuteTrajectory.Goal()
        # Get the RobotTrajectory from the motion_plan_response
        goal.trajectory = plan_resp.motion_plan_response.trajectory

        self.get_logger().info(f"Sending trajectory for execution: {label}")
        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error("Timeout while sending ExecuteTrajectory goal.")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("ExecuteTrajectory goal rejected.")
            return False

        self.get_logger().info("ExecuteTrajectory goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done():
            self.get_logger().error("ExecuteTrajectory did not finish.")
            return False

        result = result_future.result()
        error_val = result.result.error_code.val
        self.get_logger().info(
            f"Execution finished for {label} (MoveIt error_code={error_val})"
        )
        return error_val == 1
        
    def close_gripper(self):
        """Closes the Panda gripper to grasp an object."""
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Gripper action server not available!")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = 0.0        # fully closed
        goal.command.max_effort = 40.0     # strong grasp

        self.get_logger().info("Sending CLOSE gripper command...")
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        return True


    def open_gripper(self):
        """Opens the Panda gripper."""
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Gripper action server not available!")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = 0.04       # fully open (4 cm spread)
        goal.command.max_effort = 10.0

        self.get_logger().info("Sending OPEN gripper command...")
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        return True
    


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

