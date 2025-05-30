import numpy as np
import yaml

def load_map(matrix_txt_path, config_yaml_path):
    # Load grid from .txt
    with open(matrix_txt_path, "r") as f:
        grid = [list(map(int, line.strip().split())) for line in f]

    binary_grid = np.array(grid, dtype=np.uint8)
    # 0 = walkable, 1 = obstacle

    # Load tile size and origin from config
    with open(config_yaml_path, "r") as f:
        config = yaml.safe_load(f)

    resolution = config.get("tile_size", 1.0)
    origin = tuple(config["robot"]["charge_locale"])

    return binary_grid, resolution, origin
