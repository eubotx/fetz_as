import cv2
import numpy as np

class ImageRectification:
    def __init__(self, calibration_data):
        self.calibration_data = calibration_data

        print(f"ImageRectification: Resolution: {self.calibration_data['resolution']}, "
              f"Distortion Coeffs: {self.calibration_data['distortion_coeffs']}")

        if self.calibration_data['useFisheyeModel']:
            print(f"Using fisheye model for rectification.")
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.calibration_data['camera_matrix'],
                self.calibration_data['distortion_coeffs'],
                np.eye(3),
                self.calibration_data['camera_matrix'],
                self.calibration_data['resolution'][::-1],
                cv2.CV_16SC2)
        else:
            print(f"Using standard model for rectification.")
            newCameraIntrinsics = cv2.getOptimalNewCameraMatrix(
                self.calibration_data['camera_matrix'],
                self.calibration_data['distortion_coeffs'],
                self.calibration_data['resolution'][::-1],
                1, # Free scaling parameter between 0 (when all the pixels in the undistorted image are valid) and 1 (when all the source image pixels are retained in the undistorted image).
                # centerPrincipalPoint=True
            )
            print(f"Rectified camera matrix:\n{newCameraIntrinsics}")
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                cameraMatrix=self.calibration_data['camera_matrix'],
                distCoeffs=self.calibration_data['distortion_coeffs'],
                R=None,
                newCameraMatrix=newCameraIntrinsics[0],
                # newCameraMatrix=self.calibration_data['camera_matrix'],
                size=self.calibration_data['resolution'][::-1],
                m1type=cv2.CV_16SC2)
            self.calibration_data['camera_matrix'] = newCameraIntrinsics[0]
            self.calibration_data['distortion_coeffs'] = newCameraIntrinsics[1]

    def get_rectified_calibration(self):
        return self.calibration_data

    def rectify(self, frame):
        assert((frame.shape[0:2] == self.calibration_data['resolution']).all(),
               f"Frame shape {frame.shape} does not match calibration resolution {self.calibration_data['resolution']}")

        print(f"frame shape in: {frame.shape}")
        cv2.imshow("Distorted Frame", frame)
        frame = cv2.remap(frame, self.map1, self.map2,
                          interpolation=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT,
                          )
        print(f"frame shape out: {frame.shape}")
        cv2.imshow("Rectified Frame", frame)
        return frame