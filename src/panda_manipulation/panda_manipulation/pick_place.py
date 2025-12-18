import rclpy
import time
import csv
import os
from visualization_msgs.msg import Marker
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


# “Straight” gripper: keep default panda_link8 orientation (identity), however grasper was rotating.
"""
TOP_DOWN_Q = Quaternion()
TOP_DOWN_Q.x = 0.0
TOP_DOWN_Q.y = 1.0
TOP_DOWN_Q.z = 0.0
TOP_DOWN_Q.w = 0.0
"""

TOP_DOWN_Q = make_quaternion_from_euler(
    roll=-math.pi,
    pitch=0.0,
    yaw=0.0
)



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
        
        # Track which phase of planning/execution we are in
        self.current_phase = ""
        
        self.marker_pub = self.create_publisher(Marker, "/timing_marker", 10)

        
        # ==== CSV logging setup ====
        self.metrics_dir = os.path.expanduser("~/panda_ws/metrics")
        os.makedirs(self.metrics_dir, exist_ok=True)
        self.metrics_file = os.path.join(self.metrics_dir, "pickplace_metrics.csv")

        # Create file with header if doesn't exist
        if not os.path.exists(self.metrics_file):
            with open(self.metrics_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "timestamp",
                    "ik_time",
                    "plan_hover",
                    "plan_grasp",
                    "plan_lift",
                    "plan_place",
                    "exec_hover",
                    "exec_grasp",
                    "exec_lift",
                    "exec_place",
                    "pipeline_total"
                ])
         
         
         
    def publish_marker(self, text, x=0.3, y=-0.5, z=1.0):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.text = text
        self.marker_pub.publish(marker)
    def print_metrics_table(self):
        self.get_logger().info("\n======= PERFORMANCE SUMMARY =======")
        self.get_logger().info(f" IK time:               {self.metrics['ik_time']:.4f} s")
        self.get_logger().info(f" Plan hover:            {self.metrics['plan_hover']:.4f} s")
        self.get_logger().info(f" Plan grasp:            {self.metrics['plan_grasp']:.4f} s")
        self.get_logger().info(f" Plan lift:             {self.metrics['plan_lift']:.4f} s")
        self.get_logger().info(f" Plan place:            {self.metrics['plan_place']:.4f} s")
        self.get_logger().info(f" Exec hover:            {self.metrics['exec_hover']:.4f} s")
        self.get_logger().info(f" Exec grasp:            {self.metrics['exec_grasp']:.4f} s")
        self.get_logger().info(f" Exec lift:             {self.metrics['exec_lift']:.4f} s")
        self.get_logger().info(f" Exec place:            {self.metrics['exec_place']:.4f} s")
        self.get_logger().info(f" TOTAL pipeline time:   {self.metrics['pipeline_total']:.4f} s")
        self.get_logger().info("===================================")

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
    def compute_grasp_height(self, obj_z):
        APPLE_RADIUS = 0.04
        SAFETY_MARGIN = 0.015  # 1.5 cm above apple

    # Example:
    # shelf top = 0.20m
    # apple radius = 0.04 → center at z = 0.24
    # top point = 0.28
    # grasp slightly above = 0.295
        grasp_z = obj_z + APPLE_RADIUS + SAFETY_MARGIN
        hover_z = grasp_z + 0.10    # 10 cm hover
        return grasp_z, hover_z

    def run_pick_place(self):
        
        if self.busy:
            return
        if self.latest_object_pose is None:
            return
        if self.current_joint_state is None:
            return
	
        pipeline_start = time.time()
	# Reset metrics storage
        self.metrics = {
	    "ik_time": 0,
	    "plan_hover": 0,
	    "plan_grasp": 0,
	    "plan_lift": 0,
	    "plan_place": 0,
	    "exec_hover": 0,
	    "exec_grasp": 0,
	    "exec_lift": 0,
	    "exec_place": 0
	}
        self.busy = True
        obj = self.latest_object_pose
        
        


        try:
            pipeline_start = self.get_clock().now()
        # Ensure gripper is open before any motion
            self.open_gripper()
            self.get_logger().info("Gripper opened before starting motions.")
            
            self.get_logger().info(
            f"Starting pick & place pipeline for object at "
            f"x={obj.pose.position.x:.3f}, y={obj.pose.position.y:.3f},z={obj.pose.position.z:.3f}")
            grasp_z, hover_z = self.compute_grasp_height(obj.pose.position.z)
        # ------------------------
        # 1) HOVER ABOVE OBJECT
        # ------------------------
            self.current_phase = "hover"
            hover_pose = PoseStamped()
            hover_pose.header.frame_id = "world"
            hover_pose.pose.position.x = obj.pose.position.x
            hover_pose.pose.position.y = obj.pose.position.y
            #hover_pose.pose.position.z = obj.pose.position.z + 0.12
            hover_pose.pose.position.z = hover_z
            hover_pose.pose.orientation = TOP_DOWN_Q

            hover_names, hover_positions = self.compute_ik(hover_pose)
            self.current_phase = "hover"
            hover_plan = self.plan_to_joint_goal(hover_names, hover_positions)
            self.execute_plan(hover_plan, "hover over object")

        # ------------------------
        # 2) MOVE DOWN TO GRASP
        # ------------------------
            grasp_pose = PoseStamped()
            grasp_pose.header.frame_id = "world"
            grasp_pose.pose.position.x = obj.pose.position.x
            grasp_pose.pose.position.y = obj.pose.position.y
            #grasp_pose.pose.position.z = obj.pose.position.z + 0.02
            grasp_pose.pose.position.z = grasp_z
            grasp_pose.pose.orientation = TOP_DOWN_Q

            grasp_names, grasp_positions = self.compute_ik(grasp_pose)
            self.current_phase = "grasp"
            grasp_plan = self.plan_to_joint_goal(grasp_names, grasp_positions)
            self.execute_plan(grasp_plan, "descend to grasp")

        # Close gripper
            self.close_gripper()

        # ------------------------
        # 3) LIFT BACK TO HOVER
        # ------------------------
            self.current_phase = "lift"
            lift_plan = self.plan_to_joint_goal(hover_names, hover_positions)
            self.execute_plan(lift_plan, "lift object")

        # ------------------------
        # 4) MOVE TO PLACE HOVER
        # ------------------------
            place_hover = PoseStamped()
            place_hover.header.frame_id = "world"
            place_hover.pose.position.x = 0.50
            place_hover.pose.position.y = -0.30
            place_hover.pose.position.z = 0.50
            place_hover.pose.orientation = TOP_DOWN_Q

            ph_names, ph_pos = self.compute_ik(place_hover)
            ph_plan = self.plan_to_joint_goal(ph_names, ph_pos)
            self.execute_plan(ph_plan, "move above bin")

        # ------------------------
        # 5) LOWER INTO BIN
        # ------------------------
            place_pose = PoseStamped()
            place_pose.header.frame_id = "world"
            place_pose.pose.position.x = 0.50
            place_pose.pose.position.y = -0.30
            place_pose.pose.position.z = 0.33
            place_pose.pose.orientation = TOP_DOWN_Q

            p_names, p_pos = self.compute_ik(place_pose)
            p_plan = self.plan_to_joint_goal(p_names, p_pos)
            self.execute_plan(p_plan, "lower into bin")

            # Open gripper
            self.open_gripper()

        finally:
            self.busy = False
            
        # ---- pipeline total time metric ----
        pipeline_end = self.get_clock().now()
        pipeline_total = (pipeline_end - pipeline_start).nanoseconds / 1e9
        self.get_logger().info(
            f"TOTAL pick-place pipeline time: {pipeline_total:.3f} seconds"
        )
        if hasattr(self, "metrics"):
            self.metrics["pipeline_total"] = pipeline_total

        
        # Write to CSV
        with open(self.metrics_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                time.time(),
                self.metrics["ik_time"],
                self.metrics["plan_hover"],
                self.metrics["plan_grasp"],
                self.metrics["plan_lift"],
                self.metrics["plan_place"],
                self.metrics["exec_hover"],
                self.metrics["exec_grasp"],
                self.metrics["exec_lift"],
                self.metrics["exec_place"],
                self.metrics["pipeline_total"]
            ])
        self.get_logger().info(f"Metrics saved → {self.metrics_file}")
        self.print_metrics_table()


    
    """
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
            hover_pose.pose.position.z = obj.pose.position.z + 0.10  # 10 cm above
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
            grasp_pose.pose.position.z = obj.pose.position.z + 0.02 # same as YOLO pose
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
            place_hover_pose.pose.position.z = 0.50   # high above both shelf & bin
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
            place_pose.pose.position.z = 0.32   # a bit above bin top (0.30)
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
    """


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
        ik_start = time.time()
        future = self.ik_client.call_async(req)

        # Wait up to 5 seconds
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error("IK future did not complete in time.")
            return None, None

        resp = future.result()
        self.get_logger().info(f"IK result code: {resp.error_code.val}")
        
        ik_time = time.time() - ik_start
        self.metrics["ik_time"] = ik_time
        self.get_logger().info(f"IK solve time: {ik_time:.4f} seconds")

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
        plan_start = time.time()
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
        
        plan_time = time.time() - plan_start
        # auto-assign based on label passed into run_pick_place()
        if "hover" in self.current_phase:
            self.metrics["plan_hover"] = plan_time
        elif "grasp" in self.current_phase:
            self.metrics["plan_grasp"] = plan_time
        elif "lift" in self.current_phase:
            self.metrics["plan_lift"] = plan_time
        elif "place" in self.current_phase:
            self.metrics["plan_place"] = plan_time
        self.get_logger().info(f"Planning time: {plan_time:.4f} seconds")

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
        exec_start = time.time()
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
        exec_time = time.time() - exec_start
        if "hover" in label:
            self.metrics["exec_hover"] = exec_time
        elif "grasp" in label:
            self.metrics["exec_grasp"] = exec_time
        elif "lift" in label:
            self.metrics["exec_lift"] = exec_time
        elif "place" in label:
            self.metrics["exec_place"] = exec_time
        self.get_logger().info(f"Trajectory execution time: {exec_time:.4f} seconds")
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

