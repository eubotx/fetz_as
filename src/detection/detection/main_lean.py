import cv2
import numpy as np
import json
import detection.frame_source
import detection.detector
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        # Load transformation data
        self.src_points, self.matrix = self.load_transformation_data()
        self.arena_width = 450
        self.arena_height = 450

        self.options = {
            'camera_calibration_path': 'src/detection/data/USBGS720P02-L170_calibration.json',
            'publish_ros': True,
            'frame_path': 'src/detection/recordings/output_april_corner_movement.mp4',
            'webcam_id': 4,
            'webcam_save_stream_path': 'src/detection/recordings/testSeqxxx.mp4',
            'arena_tag': {'id': 2, 'family': 'tagStandard41h12'},
            'robot_tag': {'id': 12, 'family': 'tagStandard41h12'},
        }

        # Initialize detectors
        self.frame_source = detection.frame_source.GenericSource(detection.frame_source.SourceType.Webcam, options=self.options)
        self.diff_detector = detection.detector.DiffDetector()
        self.april_tag_detector = detection.detector.AprilTagDetector(self.frame_source.get_calibration())

        # ROS publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/pose', 10)

        # Run main loop
        self.timer = self.create_timer(0.03, self.process_frame)  # ~30 FPS

    def load_transformation_data(self):
        with open("src/detection/data/transformation_data.json", "r") as f:
            data = json.load(f)
        return np.float32(data["src_points"]), np.float32(data["matrix"])

    def draw_forward_direction_axis(self, frame, center, yaw, length=50):
        forward_direction = (math.cos(yaw), math.sin(yaw))
        forward_end = (
            int(center[0] + forward_direction[0] * length),
            int(center[1] - forward_direction[1] * length)
        )
        cv2.arrowedLine(frame, center, forward_end, (0, 255, 0), 5)

    def process_frame(self):
        frame = self.frame_source.get_frame()
        if frame is None:
            self.get_logger().warn("No frame captured")
            return

        transformed = cv2.warpPerspective(frame, self.matrix, (self.arena_width, self.arena_height))
        gray = cv2.cvtColor(transformed, cv2.COLOR_BGR2GRAY)

        april_tag_output = self.april_tag_detector.detect(gray, transformed.copy())
        april_tag_detections = april_tag_output["aprilTags"]

        robot_tag = self.options['robot_tag']
        for detection in april_tag_detections:
            if detection.tag_family.decode() == robot_tag['family'] and detection.tag_id == robot_tag['id']:
                center = tuple(detection.center.astype(int))
                R = detection.pose_R
                yaw = math.atan2(R[1, 0], R[0, 0])
                offset_yaw = yaw - math.pi / 2

                # Draw and debug
                robot_img = transformed.copy()
                cv2.circle(robot_img, center, 5, (0, 0, 255), -1)
                self.draw_forward_direction_axis(robot_img, center, -offset_yaw)
                cv2.putText(robot_img, f"Yaw: {offset_yaw:.2f} rad", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Robot Pose", robot_img)

                # Publish to ROS2
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = "camera_frame"
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.position.x = float(center[0])
                pose_msg.pose.position.y = float(center[1])
                pose_msg.pose.position.z = 0.0

                # Yaw as quaternion
                qz = math.sin(offset_yaw / 2.0)
                qw = math.cos(offset_yaw / 2.0)
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw

                self.pose_pub.publish(pose_msg)

        cv2.imshow("Original", frame)
        cv2.imshow("Transformed", transformed)
        if isinstance(april_tag_output, dict) and april_tag_output["debug_image"] is not None:
            cv2.imshow("AprilTags", april_tag_output["debug_image"])

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exiting...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
