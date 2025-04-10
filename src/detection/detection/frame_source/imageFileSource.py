import numpy as np
from PIL import Image
#from bitstring import options


class ImageFileSource:
    def __init__(self, options):
        image_file_path = options['frame_path']

        self.image = Image.open(image_file_path)
        if self.image is None:
            raise Exception(f"Error: Could not open image file: {image_file_path}.")

        self.image = np.asarray(self.image)
        self.frame_fetched = False

    def get_frame(self):
        if self.frame_fetched:
            print("End of stream.")
            return None
        self.frame_fetched = True
        return self.image