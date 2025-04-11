import cv2.fisheye
import numpy as np
import json
import os

class Calibration:
    def __init__(self, options: dict):
        if 'camera_calibration_path' in options.keys():
            calibration_path = options['camera_calibration_path']
            print(f"Loading calibration from: {calibration_path}")
            ext = os.path.splitext(calibration_path)[1]
            if ext == '.json':
                self.load_json(calibration_path)
            elif ext == '.npz':
                self.load_npz(calibration_path)
            else:
                raise Exception(f"Unknown filetype: {ext} for file: {calibration_path}")
        else:
            self.load_default_calibration()
        self.print_calibration()

    def get(self):
        return self.calib

    def change_to_rectified_image(self, calibration_data):
        self.calib_original = self.calib.copy()
        self.calib = calibration_data
        self.print_calibration()

    def project(self, pt3d):
        if self.calib['useFisheyeModel'] and False:
            pt2d, _ = cv2.fisheye.projectPoints(objectPoints=pt3d,
                                                rvec=np.eye(3),
                                                tvec=np.zeros(3),
                                                  K=self.calib['camera_matrix'],
                                                  D=self.calib['distortion_coeffs'],)
        else:
            pt2d, _ = cv2.projectPoints(objectPoints=pt3d,
                                        rvec=np.eye(3),
                                        tvec=np.zeros(3),
                                        cameraMatrix=self.calib['camera_matrix'],
                                        distCoeffs=self.calib['distortion_coeffs'],)
        return pt2d

    def print_calibration(self):
        for k, v in self.calib.items():
            print(f"{k}: {v}")

        res_up = -self.calib['camera_matrix'][0][2]
        res_down = self.calib['resolution'][0] - self.calib['camera_matrix'][0][2]
        res_left = -self.calib['camera_matrix'][1][2]
        res_right = self.calib['resolution'][1] - self.calib['camera_matrix'][1][2]
        def compute_fov(f, res):
            return np.arctan(res / f) * 180 / np.pi
        fx = self.calib['camera_matrix'][0][0]
        fy = self.calib['camera_matrix'][1][1]
        fov_up = compute_fov(fx, res_up)
        fov_down = compute_fov(fx, res_down)
        fov_left = compute_fov(fy, res_left)
        fov_right = compute_fov(fy, res_right)

        print(f"Camera FoV: {fov_up:.2f}° up, {fov_down:.2f}° down, {fov_left:.2f}° left, {fov_right:.2f}° right")
        fov_vertical = fov_down - fov_up
        fov_horizontal = fov_right - fov_left
        print(f"Horizontal Fov: {fov_horizontal:.2f}°, Vertical Fov: {fov_vertical:.2f}°, Diagonal Fov: {np.sqrt(fov_horizontal**2 + fov_vertical**2):.2f}°")

    def load_npz(self, npz_file):
        self.calib = np.load(npz_file)

    def load_json(self, json_file):
        # Load JSON file
        with open(json_file, 'r') as f:
            json_data = json.load(f)

        self.calib = {key: np.array(value) for key, value in json_data.items()}

    def load_default_calibration(self):
        # Default calibration data
        resolution = np.array([480, 640])
        focal_length = 300.0
        self.calib = {
            'resolution': resolution,
            'camera_matrix': np.array([[focal_length, 0, resolution[1]/2],
                                       [0, focal_length, resolution[0]/2],
                                       [0, 0, 1.0]]),
            'distortion_coeffs': np.zeros((5,)),
            'useFisheyeModel': False,
            'mean_error': -1.0,
            'max_error': -1.0,
        }

    def convert_npz_to_json(self, npz_filepath):
        # Load the NPZ file
        data = np.load(npz_filepath, allow_pickle=True)

        # Convert numpy arrays to lists
        json_data = {key: data[key].squeeze().tolist() for key in data}

        # Generate JSON file path
        json_filepath = os.path.splitext(npz_filepath)[0] + ".json"

        # Save as JSON
        with open(json_filepath, 'w') as f:
            json.dump(json_data, f, indent=4)
        print(f"JSON saved to {json_filepath}")