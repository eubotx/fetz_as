import cv2

class WebCamSource:
    def __init__(self):
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            raise Exception("Error: Could not open webcam.")

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return None

        camera_properties = {
            'width': cv2.CAP_PROP_FRAME_WIDTH,
            'height': cv2.CAP_PROP_FRAME_HEIGHT,
            'fps': cv2.CAP_PROP_FPS,
            'brightness': cv2.CAP_PROP_BRIGHTNESS,
            'contrast': cv2.CAP_PROP_CONTRAST,
            'saturation': cv2.CAP_PROP_SATURATION,
            'gain': cv2.CAP_PROP_GAIN,
            'exposure': cv2.CAP_PROP_EXPOSURE,
            'focus': cv2.CAP_PROP_FOCUS
        }
        # for prop, value in camera_properties.items():
        #     print(f"{prop}: {self.cap.get(value)}")
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, 3)
        # self.cap.set(cv2.CAP_PROP_FPS, 60)

        return frame

    def __del__(self):
        self.cap.release()