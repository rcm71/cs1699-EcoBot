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

    # New solution with multiple trips
    full_path = []
    visited_pois = []
    remaining = set(poi_list)
    
    # Keep making trips until we can't visit any more POIs
    trip_number = 1
    
    while remaining:
        print(f"\n=== Planning Trip #{trip_number} ===")
        # Start each trip from the charging station with full battery
        current = charge
        total_energy = 0
        trip_path = [current]  # Start path at charging station
        made_progress = False
        
        while remaining:
            reachable = []
            for target in remaining:
                path = dijkstra(graph, current, target)
                if not path:
                    continue
                dist = path_distance(path, tile_size)
                energy = (dist / tile_size) * energy_per_sec + (wait_time * scan_energy)
                
                # Check if we have enough energy to visit this POI and return to charging station
                return_energy_needed = return_energy(target, charge, graph, tile_size, energy_per_sec)
                
                if total_energy + energy + return_energy_needed <= battery_capacity:
                    reachable.append((energy, target, path))

            if not reachable:
                print(f"Trip #{trip_number}: No more POIs can be visited within battery budget.")
                break

            # Visit the POI with lowest energy cost
            reachable.sort()
            energy, target, path = reachable[0]

            # Add path to target (excluding current position which is already in the path)
            trip_path.extend(path[1:] if path[0] == current else path)
            visited_pois.append(target)
            remaining.remove(target)
            total_energy += energy
            current = target
            made_progress = True
            print(f"Trip #{trip_number}: Visited POI at {target}, remaining energy: {battery_capacity - total_energy:.4f}")

        # Return to charging station at the end of trip if we've visited any POIs
        if made_progress and current != charge:
            return_path = dijkstra(graph, current, charge)
            if return_path:
                # Add return path (excluding current position)
                trip_path.extend(return_path[1:])
            print(f"Trip #{trip_number}: Returned to charging station, total energy used: {total_energy:.4f}")
        
        # Add this trip to our complete solution
        if made_progress:
            full_path.extend(trip_path if not full_path else trip_path[1:])  # Don't duplicate charge station
            trip_number += 1
        else:
            # If we didn't make progress, some POIs are unreachable
            print(f"Could not reach {len(remaining)} POIs even with multiple trips.")
            break
    
    # If we didn't manage to visit any POIs
    if not full_path and charge:
        full_path = [charge]
        
    print(f"\nVisited {len(visited_pois)} out of {len(poi_list)} POIs across {trip_number-1} trips")
    print(f"Visited POIs: {visited_pois}")
    
    return full_path, visited_pois


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