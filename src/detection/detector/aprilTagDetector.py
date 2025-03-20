import cv2
import numpy as np
import pyapriltags as apriltag  # Documentation: https://github.com/WillB97/pyapriltags

class AprilTagDetector:
    def __init__(self, camera_calibration_file=None):
        if camera_calibration_file is None:
            camera_calibration_file = 'data/tiago_laptop_webcam_calibration.npz'
        with np.load(camera_calibration_file) as data:
            camera_matrix = data['camera_matrix']
            distortion_coeffs = data['distortion_coeffs']
        print(f"Loaded camera matrix:\n{camera_matrix}")
        self.camera_params=np.array([camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]])

        # Initialize the AprilTag detector
        self.detector = apriltag.Detector(families="tagStandard41h12")

    def detect(self, image_gray, debug_image=None):
        # Detect AprilTags
        detections = self.detector.detect(image_gray,
                                     estimate_tag_pose=True,
                                     camera_params=self.camera_params,
                                     tag_size=0.15)
        if debug_image is None:
            return {"aprilTags": detections}

        for detection in detections:
            # Draw a bounding box around the detected tag
            for i in range(4):
                pt1 = tuple(detection.corners[i].astype(int))
                pt2 = tuple(detection.corners[(i + 1) % 4].astype(int))
                cv2.line(debug_image, pt1, pt2, (0, 255, 0), 2)

            # Draw the center of the tag
            center = tuple(detection.center.astype(int))
            cv2.circle(debug_image, center, 5, (0, 0, 255), -1)

            # Display tag ID
            cv2.putText(debug_image, f"ID: {detection.tag_id}", (center[0] - 10, center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return {"aprilTags": detections, "debug_image": debug_image}