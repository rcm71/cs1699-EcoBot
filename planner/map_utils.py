import numpy as np
from PIL import Image
import yaml

def load_map(pgm_path, yaml_path):
    # Load metadata
    with open(yaml_path, 'r') as file:
        metadata = yaml.safe_load(file)

    resolution = metadata["resolution"]
    origin = metadata["origin"]

    # Load PGM image as numpy array
    image = Image.open(pgm_path)
    image_array = np.array(image)

    # Convert image to binary grid: 1 = free, 0 = obstacle/unknown
    binary_grid = np.zeros_like(image_array, dtype=np.uint8)
    binary_grid[image_array > 254] = 1  # 255 = free (white)

    return binary_grid, resolution, origin
