import cv2
import numpy as np

from src.detection.frame_source.imageRectification import ImageRectification

class WebCamSource:
    def __init__(self, webcam_id=0,
                 save_stream_path='/home/tiago/repos/fetz_as/src/detection/data/testSeqxxx.mp4'):
        # Open webcam
        self.cap = cv2.VideoCapture(webcam_id)
        if not self.cap.isOpened():
            raise Exception("Error: Could not open webcam.")

        # Create rectification LUT
        self.rectification = ImageRectification()

        # Optionally save webcam stream to file
        if save_stream_path is None:
            self.out = None
        else:
            fourcc = cv2.VideoWriter_fourcc(*'MP4V')
            self.out = cv2.VideoWriter(filename=save_stream_path,
                                       fourcc=fourcc,
                                       fps=self.cap.get(cv2.CAP_PROP_FPS),
                                       frameSize=np.array([self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)]).astype(int))

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return None

        if self.out is not None:
            self.out.write(frame)

        frame = self.rectification.rectify(frame)

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
        if self.out is not None:
            self.out.release()