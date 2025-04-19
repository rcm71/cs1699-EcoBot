import yaml
from planner.pathfinder import dijkstra
from planner.map_utils import load_map
from math import hypot

def load_config(yaml_path):
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)

def solve_bc_tsp(config_file, graph):
    config = load_config(config_file)

    # === Robot + map config ===
    battery_capacity = config["robot"]["battery_capacity"]
    energy_per_sec = config["robot"]["energy_consumption"]
    wait_time = config["sensor"]["time_to_wait"]
    scan_energy = config["sensor"]["energy_consuption"]
    tile_size = config.get("tile_size", 1.0)

    charge = tuple(config["robot"]["charge_locale"])
    poi_list = [tuple(p) for p in config["key_points"]]

    visited_path = []
    visited_pois = []
    remaining = set(poi_list)
    current = charge
    total_energy = 0

    while remaining:
        reachable = []
        for target in remaining:
            path = dijkstra(graph, current, target)
            if not path:
                continue
            dist = path_distance(path, tile_size)
            energy = (dist / tile_size) * energy_per_sec + (wait_time * scan_energy)
            if total_energy + energy + return_energy(target, charge, graph, tile_size, energy_per_sec) <= battery_capacity:
                reachable.append((energy, target, path))

        if not reachable:
            print("No more POIs can be visited within battery budget.")
            break

        reachable.sort()
        energy, target, path = reachable[0]

        visited_path.extend(path[1:] if visited_path else path)
        visited_pois.append(target)
        remaining.remove(target)
        total_energy += energy
        current = target

    # === Return to base ===
    if current != charge:
        return_path = dijkstra(graph, current, charge)
        if return_path:
            visited_path.extend(return_path[1:])

    print(f"Visited POIs: {visited_pois}")
    print(f"Total estimated energy used: {total_energy:.4f} kWh")

    return visited_path or [charge], visited_pois


def path_distance(path, tile_size):
    """Calculate Euclidean distance between consecutive points."""
    total = 0
    for (a_i, a_j), (b_i, b_j) in zip(path, path[1:]):
        dx = abs(a_j - b_j)
        dy = abs(a_i - b_i)
        dist = tile_size if dx == 0 or dy == 0 else tile_size * 2 ** 0.5
        total += dist
    return total

def return_energy(current, base, graph, tile_size, energy_per_sec):
    path = dijkstra(graph, current, base)
    if not path:
        return float("inf")
    dist = path_distance(path, tile_size)
    return (dist / tile_size) * energy_per_sec
