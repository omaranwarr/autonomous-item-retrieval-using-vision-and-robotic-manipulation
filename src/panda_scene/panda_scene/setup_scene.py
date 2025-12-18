import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class SceneSetup(Node):
    def __init__(self):
        super().__init__("scene_setup")

        self.pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        self.timer = self.create_timer(2.0, self.publish_scene)
    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        objects = []

    # ---------------------------
    # 1) SHELF
    # ---------------------------
        shelf = CollisionObject()
        shelf.id = "shelf"
        shelf.header.frame_id = "world"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.50, 0.30, 0.20]  # width, depth, height

        shelf_pose = Pose()
        shelf_pose.position.x = 0.60
        shelf_pose.position.y = 0.20
        shelf_pose.position.z = 0.10  # half of 0.20

        shelf.primitive_poses.append(shelf_pose)
        shelf.primitives.append(box)
        objects.append(shelf)

    # ---------------------------
    # 2) BIN
    # ---------------------------
        bin_obj = CollisionObject()
        bin_obj.id = "bin"
        bin_obj.header.frame_id = "world"

        bin_box = SolidPrimitive()
        bin_box.type = SolidPrimitive.BOX
        bin_box.dimensions = [0.25, 0.25, 0.30]

        bin_pose = Pose()
        bin_pose.position.x = 0.50
        bin_pose.position.y = -0.30
        bin_pose.position.z = 0.15   # half height

        bin_obj.primitive_poses.append(bin_pose)
        bin_obj.primitives.append(bin_box)
        objects.append(bin_obj)

    # ---------------------------
    # 3) APPLE (SPHERE)
    # ---------------------------
        apple = CollisionObject()
        apple.id = "apple"
        apple.header.frame_id = "world"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.04]   # radius 4 cm

        apple_pose = Pose()
        apple_pose.position.x = 0.60
        apple_pose.position.y = 0.20
        apple_pose.position.z = 0.20 + 0.04   # shelf_height + radius

        apple.primitive_poses.append(apple_pose)
        apple.primitives.append(sphere)
        objects.append(apple)

        scene.world.collision_objects = objects
        scene.robot_state.is_diff = True
        self.pub.publish(scene)

        self.get_logger().info("Published shelf, bin, and apple with corrected dimensions.")

"""
    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        objects = []

        # ---------------------------
        # 1) S H E L F 
        # ---------------------------
        shelf = CollisionObject()
        shelf.id = "shelf"
        shelf.header.frame_id = "world"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.5, 0.3, 0.20]   # width, depth, height of shelf

        shelf_pose = Pose()
        shelf_pose.position.x = 0.60
        shelf_pose.position.y = 0.20
        shelf_pose.position.z = 0.1      # half height -> 0.40 / 2

        shelf.primitive_poses.append(shelf_pose)
        shelf.primitives.append(box)

        objects.append(shelf)

        # ---------------------------
        # 2) B I N  (place location)
        # ---------------------------
        bin_obj = CollisionObject()
        bin_obj.id = "bin"
        bin_obj.header.frame_id = "world"

        bin_box = SolidPrimitive()
        bin_box.type = SolidPrimitive.BOX
        bin_box.dimensions = [0.25, 0.25, 0.30]

        bin_pose = Pose()
        bin_pose.position.x = 0.50
        bin_pose.position.y = -0.30
        bin_pose.position.z = 0.15   # half height

        bin_obj.primitive_poses.append(bin_pose)
        bin_obj.primitives.append(bin_box)

        objects.append(bin_obj)

        # ---------------------------
        # 3) A P P L E  (YOLO item)
        # ---------------------------
        apple = CollisionObject()
        apple.id = "apple"
        apple.header.frame_id = "world"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.04]    # radius 4 cm

        apple_pose = Pose()
        apple_pose.position.x = 0.60
        apple_pose.position.y = 0.20

        # -----------------------------------------
        # FIXED: Apple sits on shelf, not floating
        # shelf top = 0.2 m, apple radius = 0.04 m
        # apple center = 0.2 + 0.04 = 0.24
        # -----------------------------------------
        apple_pose.position.z = 0.24  

        apple.primitive_poses.append(apple_pose)
        apple.primitives.append(sphere)

        objects.append(apple)

        # Add all objects
        scene.world.collision_objects = objects
        scene.robot_state.is_diff = True

        self.pub.publish(scene)
        self.get_logger().info("Published scene objects (shelf, bin, apple)")
"""

def main(args=None):
    rclpy.init(args=args)
    node = SceneSetup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

