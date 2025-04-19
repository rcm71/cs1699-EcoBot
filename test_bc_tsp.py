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

    config = load_config(config_file)

    def check_valid(pt):
        x, y = pt 
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

    # Find the charging trips (segments between returns to charging station)
    charging_trips = []
    current_trip = []
        
    for i, point in enumerate(path):
        current_trip.append(point)
        # If we're at the charging station and not at the start or end
        if point == charge and i > 0 and i < len(path) - 1:
            charging_trips.append(current_trip)
            current_trip = [point]  # Start a new trip from the charging station
        
    # Add the last trip if it's not empty
    if current_trip:
        charging_trips.append(current_trip)
        
    # If we don't have any complete trips (just one continuous path), treat it as one trip
    if not charging_trips:
        charging_trips = [path]

    # Determine which POIs are unreachable
    unreachable_pois = [poi for poi in poi_list if poi not in visited_pois]



    plt.imshow(1 - binary_grid, cmap='gray')
    trip_colors = ['skyblue', 'orange', 'purple', 'lime', 'deepskyblue', 'salmon']
        
    for i, trip in enumerate(charging_trips):
        color = trip_colors[i % len(trip_colors)]
        label = f"Trip {i+1}"
        
        # Get x,y coordinates for plotting
        trip_x, trip_y = zip(*trip)
        plt.plot(trip_x, trip_y, color=color, linewidth=2, label=label)

    # Plot visited POIs in red
    if visited_pois:
        poi_x, poi_y = zip(*visited_pois)
        plt.scatter(poi_x, poi_y, c='red', s=50, label="Visited POIs")

    # Plot unreachable POIs in gray
    if unreachable_pois:
        unreached_x, unreached_y = zip(*unreachable_pois)
        plt.scatter(unreached_x, unreached_y, c='silver', s=50, label="Unreachable POIs")

    # Plot charge station in green
    charge_x, charge_y = charge
    plt.scatter(charge_x, charge_y, c='green', s=80, marker='*', label="Charge Station")

    
    plt.title("POIs on Binary Map")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.grid(True)
    plt.show()

    return path, visited_pois, charging_trips, unreachable_pois, charge


if __name__ == "__main__":
    main()