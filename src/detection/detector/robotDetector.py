
import numpy as np
import cv2

from src.detection.math.pose import Pose
from src.detection.calibration.calibration import Calibration
from src.detection.detector import AprilTagDetector


arrow_forward_x = 0.2 * np.array([[0, 0, 0],
                [1, 0, 0],
                [0.75, 0.25, 0],
                [1, 0, 0],
                [0.75, -0.25, 0],
                [1, 0, 0]])


def draw_outline(outline, image, text=None, color=(0, 255, 0)):
    for i in range(len(outline)):
        pt1 = tuple(outline[i].astype(int))
        pt2 = tuple(outline[(i + 1) % len(outline)].astype(int))
        cv2.line(image, pt1, pt2, color, 2)
        cv2.putText(image, f"{i}", pt1,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    if text:
        cv2.putText(image, text, outline[0].astype(int),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

class RobotDetector():
    def __init__(self, options: dict, calibration: Calibration):
        self.robot_tags = options['robot_tags']
        self.robot_outline = 0.15 * np.array([[0,-4,0],
                                              [2,0,0],
                                              [2, 1, 0],
                                              [-2, 1, 0],
                                              [-2, 0, 0]])
        self.detection_last = None
        self.calibration = calibration
        self.april_tag_detector = AprilTagDetector(self.calibration)

    def detect(self, image_gray, worldFromCamera: Pose, debug_image=None):
        # Detect Robot
        print(f'Detecting robot...')

        april_tag_detections = self.april_tag_detector.detect(image_gray, self.robot_tags['sizes'])['aprilTags']
        for tag_detection in april_tag_detections:
            print(f'robotDetector: detected tag {tag_detection.tag_id} of family {tag_detection.tag_family.decode()} with pose error {tag_detection.pose_err:.3f} m')

        for tag_detection in april_tag_detections:
            if (tag_detection.tag_family.decode() == self.robot_tags['family'] and
                    (tag_detection.tag_id == self.robot_tags['top_id'] or
                     tag_detection.tag_id == self.robot_tags['bottom_id'])):
                if tag_detection.pose_err > 1e-3:
                    print(f"Detection error is large: {tag_detection.pose_err:.3f} m")

                detection = {}
                detection['cameraFromRobot'] = Pose(tag_detection.pose_R, tag_detection.pose_t)
                detection['robotInCamera2D'] = self.calibration.project(tag_detection.pose_t).squeeze()
                detection['worldFromRobot'] = worldFromCamera * detection['cameraFromRobot']

                # Compute the robot forward arrow in 2D
                robotForwardArrowInCamera2D = []
                for arrow_point in arrow_forward_x:
                    arrowPoseInRobot = Pose(np.eye(3), arrow_point)
                    robotForwardArrowInCamera2D.append(self.calibration.project(
                        (detection['cameraFromRobot'] * arrowPoseInRobot).t).squeeze())

                # Compute the robot outline in 3D and 2D
                robotOutlineInCamera3D = []
                robotOutlineInCamera2D = []
                robotOutlineInWorld3D = []

                for outline_point in self.robot_outline:
                    outlineInRobot = Pose(np.eye(3), outline_point)
                    robotOutlineInCamera3D.append(detection['cameraFromRobot'] * outlineInRobot)
                    robotOutlineInCamera2D.append(self.calibration.project(robotOutlineInCamera3D[-1].t).squeeze())
                    robotOutlineInWorld3D.append(detection['worldFromRobot'] * outlineInRobot)
                detection['robotOutlineInCamera2D'] = robotOutlineInCamera2D

                # Compute the tag outline in 3D and 2D
                tag_outline = self.april_tag_detector.tag_outline()
                tagOutlineInCamera3D = []
                tagOutlineInCamera2D = []

                for outline_point in tag_outline:
                    outlineInRobot = Pose(np.eye(3), outline_point)
                    tagOutlineInCamera3D.append(detection['cameraFromRobot'] * outlineInRobot)
                    tagOutlineInCamera2D.append(self.calibration.project(tagOutlineInCamera3D[-1].t).squeeze())

                if debug_image is not None:
                    # self.april_tag_detector.draw_in_image(detection, debug_image)
                    # draw_outline(tagOutlineInCamera2D, debug_image, 'tagProjected', (0, 255, 0))
                    cv2.circle(debug_image, detection['robotInCamera2D'].astype(int), 5, (0, 255, 0))
                    draw_outline(robotForwardArrowInCamera2D, debug_image, 'forward', (255, 0, 0))
                    draw_outline(robotOutlineInCamera2D, debug_image, 'robot', (0, 255, 0))

                    cv2.imshow("RobotDetector Debug Image", debug_image)

                self.detection_last = detection
                return detection

        print(f'RobotDetector: Could not find robot')
        if debug_image is not None:
            if self.detection_last is not None:
                draw_outline(self.detection_last['robotOutlineInCamera2D'] , debug_image, 'robot', (0, 0, 255))
            cv2.imshow("RobotDetector Debug Image", debug_image)
        return self.detection_last