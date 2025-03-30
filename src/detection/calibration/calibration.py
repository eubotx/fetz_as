import numpy as np
import json
import os

from numpy.testing.print_coercion_tables import print_cancast_table
from scipy.sparse import load_npz


class Calibration:
    def __init__(self, calibration_path):
        ext = os.path.splitext(calibration_path)[1]
        if ext == '.json':
            self.load_json(calibration_path)
        elif ext == '.npz':
            self.load_npz(calibration_path)
        else:
            raise Exception(f"Unknown filetype: {ext} for file: {calibration_path}")

    def get(self):
        return self.calib

    def print_calibration(self):
        for k, v in self.calib.items():
            print(f"{k}: {v}")

    def load_npz(self, npz_file):
        self.calib = np.load(npz_file)
        self.print_calibration()

    def load_json(self, json_file):
        # Load JSON file
        with open(json_file, 'r') as f:
            json_data = json.load(f)

        self.calib = {key: np.array(value) for key, value in json_data.items()}
        self.print_calibration()

    def convert_npz_to_json(self, npz_filepath):
        # Load the NPZ file
        data = np.load(npz_filepath, allow_pickle=True)

        # Convert numpy arrays to lists
        json_data = {key: data[key].tolist() for key in data}

        # Generate JSON file path
        json_filepath = os.path.splitext(npz_filepath)[0] + ".json"

        # Save as JSON
        with open(json_filepath, 'w') as f:
            json.dump(json_data, f, indent=4)
        print(f"JSON saved to {json_filepath}")