import cv2
import numpy as np

from frame_source import VideoFileSource, WebCamSource, ImageFileSource
from calibration.calibration import Calibration
from frame_source.imageRectification import ImageRectification

from enum import Enum
class SourceType(Enum):
    Video = 1
    Webcam = 2
    Image = 3

class GenericSource:
    def __init__(self, source_type, options = {}):
        self.options = options
        if source_type == SourceType.Video:
            self.source = VideoFileSource(options)
        elif source_type == SourceType.Webcam:
            self.source = WebCamSource(options)
        elif source_type == SourceType.Image:
            self.source = ImageFileSource(options)
        else:
            raise Exception(f"Unknown source_type: {source_type}")

        self.calibration = Calibration(options)
        if 'camera_calibration_path' in options.keys():
            self.rectification = ImageRectification(self.calibration.get())
            self.calibration.change_to_rectified_image(self.rectification.get_rectified_calibration())
        else:
            self.rectification = None

        self.frame_number = 0

    def get_frame(self):
        frame = self.source.get_frame()
        if frame is None:
            return None
        self.frame_number += 1
        while 'skip_frames' in self.options.keys():
            if not self.frame_number % self.options['skip_frames']:
                break
            print(f"Skipped frame: {self.frame_number}")
            frame = self.source.get_frame()
            self.frame_number += 1

        #print(f"Frame size: {frame.shape}")
        if self.rectification is not None:
            frame = self.rectification.rectify(frame)
            #print(f"Rectified image size: {frame.shape}")

        if 'downscale' in self.options.keys():
            height, width, _ = frame.shape
            dim = np.array([width / self.options['downscale'], height / self.options['downscale']]).astype(int)
            frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
            print(f"Downscaled frame size: {frame.shape}")

        return frame

    def get_calibration(self):
        return self.calibration