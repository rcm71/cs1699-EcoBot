from planner.map_utils import load_map
from planner.graph_builder import grid_to_graph
from planner.bc_tsp import solve_bc_tsp, load_config
import matplotlib.pyplot as plt

def main():
    map_file = "grid_txt/map_paddedMatrix.txt"
    config_file = "demo.yaml"

    binary_grid, resolution, origin = load_map(map_file, config_file)
    graph = grid_to_graph(binary_grid, resolution)

    path, visited_pois = solve_bc_tsp(config_file, graph)
    print("\nFinal path to follow:")
    for step in path:
        print(step)

    # ✅ Validate charging location and POIs
    config = load_config(config_file)

    def check_valid(pt):
        y, x = pt  # flipped for row/col access
        return (
            0 <= y < binary_grid.shape[0] and
            0 <= x < binary_grid.shape[1] and
            binary_grid[y][x] == 0
        )

    charge = tuple(config["robot"]["charge_locale"])
    print(f"\nCharge location: {charge}")
    print(f"Is charge valid? {check_valid(charge)}")

    poi_list = [tuple(p) for p in config["key_points"]]
    for i, pt in enumerate(poi_list):
        print(f"POI {i+1} at {pt} valid? {check_valid(pt)}")

    print("Binary grid shape:", binary_grid.shape)  # (rows, cols)


    # Plot the binary map
    plt.imshow(1 - binary_grid, cmap='gray')

    # Plot full path in light blue - no conversion needed, keep y,x as is
    if path:
        py_path, px_path = zip(*path)  # Already in y,x format
        plt.plot(px_path, py_path, 'skyblue', linewidth=2, label="Path")

    # Plot POIs in red - no conversion needed
    if visited_pois:
        py_poi, px_poi = zip(*visited_pois)  # Already in y,x format
        plt.scatter(px_poi, py_poi, c='red', s=50, label="Visited POIs")

    # Plot charge station in green - charge is already in y,x format
    charge_y, charge_x = charge
    plt.scatter(charge_x, charge_y, c='green', s=80, marker='*', label="Charge Station")

    plt.title("POIs on Binary Map")
    plt.xlabel("X (cols)")
    plt.ylabel("Y (rows)")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()