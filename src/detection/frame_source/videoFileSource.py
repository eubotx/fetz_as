import cv2
import numpy as np


class VideoFileSource:
    def __init__(self, video_file_path=None, frame_rate=30, start_time_sec=None):
        if video_file_path is None:
            video_file_path = "/home/tiago/repos/robofetz_2025/data/robofetz_2024.mp4"
        if start_time_sec is None:
            start_time_sec = 5 * 60 + 8

        self.cap = cv2.VideoCapture(video_file_path)
        self.cap.set(cv2.CAP_PROP_POS_MSEC, start_time_sec * 1000)
        self.fps_original = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Original Frame rate: {self.fps_original}")

        self.cap.set(cv2.CAP_PROP_FPS, frame_rate)  # TODO(@tmptorres): Seems like setting the frame rate like this does not work
        self.fps_set = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Set Frame rate: {self.fps_set}")

        if not self.cap.isOpened():
            raise Exception(f"Error: Could not open video file: {video_file_path}.")
        self.frame_number = 0

    def get_frame(self, delay_ms=0, skip_frames=5):
        if cv2.waitKey(delay_ms) == ord('c'):
            pass

        while True and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame.")
                return None
            self.frame_number += 1
            if not self.frame_number % skip_frames:
                height, width, _ = frame.shape
                dim = np.array([width/4, height/4]).astype(int)
                resized = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
                return resized
            print(f"Skipped frame: {self.frame_number}")
        print(f"End of video stream.")
        return None

    def __del__(self):
        self.cap.release()