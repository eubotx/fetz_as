import timeit
import cv2
import numpy as np

class ImageRectification:
    def __init__(self, calibration_data):
        self.calibration_data = calibration_data.copy()

        print(f"ImageRectification: Resolution: {self.calibration_data['resolution']}, "
              f"Distortion Coeffs: {self.calibration_data['distortion_coeffs']}")

        if self.calibration_data['useFisheyeModel']:
            print(f"Using fisheye model for rectification.")
            P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K=self.calibration_data['camera_matrix'],
                D=self.calibration_data['distortion_coeffs'],
                image_size=self.calibration_data['resolution'],
                R=None,
                P=None,
                balance=1, # Sets the new focal length in range between the min focal length and the max focal length. Balance is in range of [0, 1].
                new_size=self.calibration_data['resolution'],
                fov_scale=1 # Divisor for new focal length.
            )
            print(f"Rectified camera matrix:\n{P}")
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                K=self.calibration_data['camera_matrix'],
                D=self.calibration_data['distortion_coeffs'],
                R=np.eye(3),
                P=P,
                size=self.calibration_data['resolution'],
                m1type=cv2.CV_16SC2)
            self.calibration_data['camera_matrix'] = P
            self.calibration_data['distortion_coeffs'] = np.zeros(4)
        else:
            print(f"Using standard model for rectification.")
            new_camera_matrix, validPixROI = cv2.getOptimalNewCameraMatrix(
                cameraMatrix=self.calibration_data['camera_matrix'],
                distCoeffs=self.calibration_data['distortion_coeffs'],
                imageSize=self.calibration_data['resolution'],
                alpha=0, # Free scaling parameter between 0 (when all the pixels in the undistorted image are valid) and 1 (when all the source image pixels are retained in the undistorted image).
                newImgSize=self.calibration_data['resolution'],
                # centerPrincipalPoint=True,
            )
            print(f"Rectified camera matrix:\n{new_camera_matrix}, validPixROI: {validPixROI}")
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                cameraMatrix=self.calibration_data['camera_matrix'],
                distCoeffs=self.calibration_data['distortion_coeffs'],
                R=None,
                newCameraMatrix=new_camera_matrix,
                # newCameraMatrix=self.calibration_data['camera_matrix'],
                size=self.calibration_data['resolution'],
                m1type=cv2.CV_16SC2,
            )
            self.calibration_data['camera_matrix'] = new_camera_matrix
            self.calibration_data['distortion_coeffs'] = np.zeros(5)
            self.calibration_data['valid_pixel_roi'] = validPixROI

    def get_rectified_calibration(self):
        return self.calibration_data

    def rectify(self, frame):
        assert np.array_equal(frame.shape[:2][::-1],self.calibration_data['resolution']),\
            f"Frame shape {frame.shape[0:2]} does not match calibration resolution {self.calibration_data['resolution']}"

        # cv2.imshow("Distorted Frame", frame)
        start_time = timeit.default_timer()
        frame = cv2.remap(frame, self.map1, self.map2,
                          interpolation=cv2.INTER_LINEAR,
                          # interpolation=cv2.INTER_CUBIC,
                          borderMode=cv2.BORDER_CONSTANT,
                          )
        print(f"Time taken for rectification: {1e3*(timeit.default_timer() - start_time):.3f} ms")
        # cv2.imshow("Rectified Frame", frame)
        return frame