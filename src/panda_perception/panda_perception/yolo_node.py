import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO
import cv2
import time


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        # Load YOLO model
        self.model = YOLO("yolov8n.pt")

        # Publish object pose
        self.pose_pub = self.create_publisher(
            PoseStamped, "/detected_object_pose", 10
        )

        # Parameter: image path
        self.declare_parameter(
            "image_path", "/home/robot7/item.jpg"
        )

        # Run detection every 2 seconds
        self.timer = self.create_timer(2.0, self.detect_object)

    def detect_object(self):
        image_path = self.get_parameter("image_path").value
        img = cv2.imread(image_path)

        if img is None:
            self.get_logger().error(f"Could not load image {image_path}")
            return

        # YOLO inference
        start = time.time()
        results = self.model(img, verbose=False)
        yolo_time = time.time() - start
        self.get_logger().info(f"YOLO inference time: {yolo_time:.4f} sec")

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                label = self.model.names[cls]

                self.get_logger().info(f"Detected object: {label}")

                # Approximate 3D pose on the shelf
                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose.position.x = 0.60
                pose.pose.position.y = 0.20
                pose.pose.position.z = 0.30  # a bit above shelf height
                
                # Top-down grasp orientation (gripper pointing straight downward)
                pose.pose.orientation.x = 1.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 0.0

                self.pose_pub.publish(pose)
                return  # Only need first detection


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

