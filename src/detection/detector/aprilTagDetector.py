import cv2
import numpy as np
import pyapriltags as apriltag  # Documentation: https://github.com/WillB97/pyapriltags

from src.detection.calibration.calibration import Calibration

class AprilTagDetector:
    def __init__(self, options):
        if 'camera_calibration_path' in options.keys():
            camera_matrix = Calibration(options['camera_calibration_path']).get()['camera_matrix']
        else:
            camera_matrix = np.eye(3)
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        self.camera_intrinsics=np.array([fx, fy, cx, cy])

        # Initialize the AprilTag detector
        self.detector = apriltag.Detector(families="tagStandard41h12")

    def detect(self, image_gray, debug_image=None):
        # Detect AprilTags
        detections = self.detector.detect(image_gray,
                                     estimate_tag_pose=True,
                                     camera_params=self.camera_intrinsics,
                                     tag_size=0.15)
        if debug_image is None:
            return {"aprilTags": detections}

        for detection in detections:
            # Draw a bounding box around the detected tag
            for i in range(4):
                pt1 = tuple(detection.corners[i].astype(int))
                pt2 = tuple(detection.corners[(i + 1) % 4].astype(int))
                cv2.line(debug_image, pt1, pt2, (0, 255, 0), 2)
                cv2.putText(debug_image, f"{i}", pt1,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # Draw the center of the tag
            center = tuple(detection.center.astype(int))
            cv2.circle(debug_image, center, 5, (0, 0, 255), -1)

            # Display tag ID
            cv2.putText(debug_image, f"ID: {detection.tag_id}", (center[0] - 10, center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return {"aprilTags": detections, "debug_image": debug_image}