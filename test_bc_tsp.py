from planner.map_utils import load_map
from planner.graph_builder import grid_to_graph
from planner.bc_tsp import solve_bc_tsp
import matplotlib.pyplot as plt
from planner.bc_tsp import solve_bc_tsp, load_config
from PIL import Image




def main():
    map_file = "map.pgm"
    yaml_file = "map.yaml"
    config_file = "demo.yaml"   # <-- Replace with the config name you used

    # Load map and grid
    binary_grid, resolution, origin = load_map(map_file, yaml_file)
    graph = grid_to_graph(binary_grid, resolution)

    # Run the planner
    path = solve_bc_tsp(config_file, graph)

    print("\nFinal path to follow:")
    for step in path:
        print(step)
    
    print(f"\nCharge location: {config_file}")
    config = load_config(config_file)
    binary_grid, resolution, origin = load_map(map_file, yaml_file)

    def check_valid(pt):
        y, x = pt  # flipped!
        return (
            0 <= y < binary_grid.shape[0] and
            0 <= x < binary_grid.shape[1] and
            binary_grid[y][x] == 1
        )

    charge = tuple(config["robot"]["charge_locale"])
    print(f"Is charge valid? {check_valid(charge)}")

    for i, pt in enumerate(config["key_points"]):
        print(f"POI {i+1} at {pt} valid? {check_valid(pt)}")


    # Load map image for background
    img = Image.open("map_padded.png")

    plt.imshow(img, cmap='gray')

    # Flip to (x, y) format correctly for matplotlib
    coords = [tuple(config["robot"]["charge_locale"])] + [tuple(p) for p in config["key_points"]]
    px, py = zip(*[(x, y) for (y, x) in coords])  # flip for correct orientation

    plt.plot(px, py, 'ro')  # x=cols, y=rows
    plt.title("POIs Overlaid on Map")
    plt.show()


if __name__ == "__main__":
    main()
