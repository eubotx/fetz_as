import cv2
import numpy as np

from src.detection.calibration.calibration import Calibration

class ImageRectification:
    def __init__(self, options):
        self.calibration_data = Calibration(options['camera_calibration_path']).get()

        if self.calibration_data['useFisheyeModel']:
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.calibration_data['camera_matrix'],
                self.calibration_data['distortion_coeffs'].squeeze(),
                np.eye(3),
                self.calibration_data['camera_matrix'],
                self.calibration_data['resolution'].squeeze()[0:2],
                cv2.CV_16SC2)
        else:
            rectifiedCamMatrix = cv2.getOptimalNewCameraMatrix(self.calibration_data['camera_matrix'],
                                          self.calibration_data['distortion_coeffs'].squeeze(),
                                          self.calibration_data['resolution'].squeeze()[::-1],
                                          1, # Free scaling parameter between 0 (when all the pixels in the undistorted image are valid) and 1 (when all the source image pixels are retained in the undistorted image).
                                           centerPrincipalPoint=True
                                        )
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.calibration_data['camera_matrix'],
                self.calibration_data['distortion_coeffs'].squeeze(),
                np.eye(3),
                rectifiedCamMatrix[0],
                self.calibration_data['resolution'].squeeze()[::-1],
                cv2.CV_16SC2)

    def rectify(self, frame):
        assert((frame.shape[0:2] == self.calibration_data['resolution'][0:2]).all())

        print(f"frame shape in: {frame.shape}")
        cv2.imshow("Distorted Frame", frame)
        frame = cv2.remap(frame, self.map1, self.map2,
                          interpolation=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT)
        print(f"frame shape out: {frame.shape}")
        cv2.imshow("Rectified Frame", frame)
        return frame