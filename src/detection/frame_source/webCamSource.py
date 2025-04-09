import cv2
import numpy as np

from src.detection.frame_source.imageRectification import ImageRectification

class WebCamSource:
    def __init__(self, options):
        # Open webcam
        self.cap = cv2.VideoCapture(options['webcam_id'])
        if not self.cap.isOpened():
            raise Exception("Error: Could not open webcam.")

        # Optionally save webcam stream to file
        if 'webcam_save_stream_path' in options.keys():
            fourcc = cv2.VideoWriter_fourcc(*'MP4V')
            self.out = cv2.VideoWriter(filename=options['webcam_save_stream_path'],
                                       fourcc=fourcc,
                                       fps=self.cap.get(cv2.CAP_PROP_FPS),
                                       frameSize=np.array([self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)]).astype(int))
        else:
            self.out = None

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return None

        if self.out is not None:
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

        return frame

    def __del__(self):
        self.cap.release()
        if self.out is not None:
            self.out.release()