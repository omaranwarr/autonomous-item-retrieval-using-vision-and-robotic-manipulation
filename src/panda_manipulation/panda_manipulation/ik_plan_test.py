import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    RobotState,
)


class IKPlanTestNode(Node):
    def __init__(self) -> None:
        super().__init__("ik_plan_test_node")

        # Latest joint state from /joint_states
        self.current_joint_state = None

        # Subscribe to joint states
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        # Service clients
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")

        # Flag so we only run the pipeline once
        self.pipeline_started = False

        # Timer to trigger the pipeline once everything is ready
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("IKPlanTestNode started. Waiting for joint states and services...")

    def joint_state_callback(self, msg: JointState) -> None:
        self.current_joint_state = msg

    def timer_callback(self) -> None:
        # Only run once
        if self.pipeline_started:
            return

        # Require joint state
        if self.current_joint_state is None:
            self.get_logger().info("No joint state yet from /joint_states, waiting...")
            return

        # Ensure services are ready
        if not self.ik_client.service_is_ready():
            self.get_logger().info("/compute_ik not ready yet, waiting...")
            return

        if not self.plan_client.service_is_ready():
            self.get_logger().info("/plan_kinematic_path not ready yet, waiting...")
            return

        self.pipeline_started = True
        self.get_logger().info("All prerequisites ready. Starting IK + planning test...")

        # Build target pose (same as you used from CLI)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = 0.6
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.32
        target_pose.pose.orientation.w = 1.0

        # ---- IK REQUEST (NO blocking, NO timeouts) ----
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "panda_arm"
        ik_req.ik_request.pose_stamped = target_pose
        ik_req.ik_request.avoid_collisions = False

        # IMPORTANT: behave like your working CLI call:
        # do NOT set robot_state here; let MoveIt use its internal state
        # (this avoids "empty JointState" being our fault).

        self.get_logger().info("Calling /compute_ik...")
        ik_future = self.ik_client.call_async(ik_req)
        ik_future.add_done_callback(self.ik_done_callback)

    def ik_done_callback(self, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f"/compute_ik call failed: {exc}")
            return

        error_code = resp.error_code.val
        self.get_logger().info(f"IK result code: {error_code}")

        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"IK failed with error code {error_code}, aborting.")
            return

        js = resp.solution.joint_state
        joint_names = list(js.name)
        joint_positions = list(js.position)

        self.get_logger().info(
            f"IK solution received with {len(joint_names)} joints. "
            "Calling /plan_kinematic_path..."
        )

        # ---- PLAN REQUEST (again async, no blocking) ----
        mreq = MotionPlanRequest()
        mreq.group_name = "panda_arm"

        # Start state: use current joint state from robot
        start_state = RobotState()
        start_state.joint_state = self.current_joint_state
        mreq.start_state = start_state

        constraints = Constraints()
        for name, pos in zip(joint_names, joint_positions):
            if not name.startswith("panda_joint"):
                continue
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        mreq.goal_constraints.append(constraints)
        mreq.num_planning_attempts = 10
        mreq.allowed_planning_time = 5.0

        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = mreq

        plan_future = self.plan_client.call_async(plan_req)
        plan_future.add_done_callback(self.plan_done_callback)

    def plan_done_callback(self, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f"/plan_kinematic_path call failed: {exc}")
            return

        error_code = resp.motion_plan_response.error_code.val
        self.get_logger().info(f"Planning result code: {error_code}")

        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Planning failed with error code {error_code}. "
                "At least now we know the pipeline runs end-to-end."
            )
            return

        self.get_logger().info("Planning succeeded! (Trajectory not executed here.)")


def main(args=None):
    rclpy.init(args=args)
    node = IKPlanTestNode()

    # Block until services are ready before spinning
    node.get_logger().info("Waiting for /compute_ik...")
    node.ik_client.wait_for_service()
    node.get_logger().info("Waiting for /plan_kinematic_path...")
    node.plan_client.wait_for_service()
    node.get_logger().info("Both services available. Spinning node.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ik_plan_test_node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

