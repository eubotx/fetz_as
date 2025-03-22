import cv2
import numpy as np

class WebCamSource:
    def __init__(self):
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            raise Exception("Error: Could not open webcam.")

        # import camera calibration data
        self.calibration_data = None
        with np.load('data/USBGS720P02-L170_calibration.npz') as data:
            for k, v in data.items():
                print(f"{k}: {v}")
            self.calibration_data = data

            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.calibration_data['camera_matrix'],
                self.calibration_data['distortion_coeffs'],
                np.eye(3),
                self.calibration_data['camera_matrix'],
                self.calibration_data['resolution'].squeeze()[0:2],
                cv2.CV_16SC2)

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        self.out = cv2.VideoWriter('/home/tiago/repos/fetz_as/src/detection/data/testSeqxxx.mp4', fourcc, 30.0, (640, 480))

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return None

        # Undistort image
        frame = cv2.remap(frame, self.map1, self.map2,
                          interpolation=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT)

        self.out.write(frame)

        # camera_properties = {
        #     'width': cv2.CAP_PROP_FRAME_WIDTH,
        #     'height': cv2.CAP_PROP_FRAME_HEIGHT,
        #     'fps': cv2.CAP_PROP_FPS,
        #     'brightness': cv2.CAP_PROP_BRIGHTNESS,
        #     'contrast': cv2.CAP_PROP_CONTRAST,
        #     'saturation': cv2.CAP_PROP_SATURATION,
        #     'gain': cv2.CAP_PROP_GAIN,
        #     'exposure': cv2.CAP_PROP_EXPOSURE,
        #     'focus': cv2.CAP_PROP_FOCUS
        # }
        # for prop, value in camera_properties.items():
        #     print(f"{prop}: {self.cap.get(value)}")
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, 3)
        # self.cap.set(cv2.CAP_PROP_FPS, 60)

        # Save frame to video file

        return frame

    def __del__(self):
        self.cap.release()
        self.out.release()