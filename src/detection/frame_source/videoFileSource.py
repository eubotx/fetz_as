import cv2
import numpy as np

from src.detection.frame_source.imageRectification import ImageRectification

class VideoFileSource:
    def __init__(self, options, downscale=None):
        self.cap = cv2.VideoCapture(options['frame_path'])
        if 'start_time_sec' in options.keys():
            self.cap.set(cv2.CAP_PROP_POS_MSEC, options['start_time_sec'] * 1000)
        self.fps_original = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Original Frame rate: {self.fps_original}")
        self.downscale = downscale

        if not self.cap.isOpened():
            raise Exception(f"Error: Could not open video file: {options['frame_path']}.")

    def get_frame(self, delay_ms=0):
        if cv2.waitKey(delay_ms) == ord('c'):
            pass

        ret, frame = self.cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return None
        return frame

    def __del__(self):
        self.cap.release()