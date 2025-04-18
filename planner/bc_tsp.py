import yaml
from planner.pathfinder import dijkstra

def load_config(config_path):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def compute_energy(distance, move_energy_rate, scan_energy_rate, time_to_wait):
    move_energy = distance * move_energy_rate
    scan_energy = scan_energy_rate * time_to_wait
    return move_energy + scan_energy

def solve_bc_tsp(config_path, graph):
    config = load_config(config_path)

    # Parse config
    battery_capacity = config["robot"]["battery_capacity"]
    move_energy_rate = config["robot"]["energy_consumption"]
    scan_energy_rate = config["sensor"]["energy_consuption"]
    time_to_wait = config["sensor"]["time_to_wait"]
    map_name = config["map_name"]

    # Locations
    charge = tuple(config["robot"]["charge_locale"])
    poi_list = [tuple(pt) for pt in config["key_points"]]  # convert all to tuples

    visited = []
    remaining = set(poi_list)
    total_energy = 0
    full_path = []
    current_pos = charge

    # Return to base at every step, so each trip is a round-trip from charge
    while remaining:
        best_poi = None
        best_cost = float('inf')
        best_path = None

        for poi in remaining:
            to_poi, cost1 = dijkstra(graph, current_pos, poi, return_cost=True)
            to_base, cost2 = dijkstra(graph, poi, charge, return_cost=True)

            if to_poi is None or to_base is None:
                continue

            total_cost = compute_energy(cost1 + cost2, move_energy_rate, scan_energy_rate, time_to_wait)
            if total_energy + total_cost <= battery_capacity and total_cost < best_cost:
                best_poi = poi
                best_cost = total_cost
                best_path = to_poi + to_base[1:]

        if best_poi is None:
            print("No more POIs can be visited within battery budget.")
            break

        visited.append(best_poi)
        remaining.remove(best_poi)
        total_energy += best_cost
        full_path.extend(best_path[:-1])  # avoid double-counting charge node
        current_pos = charge  # reset for next round

    full_path.append(charge)  # end at base
    print(f"Visited POIs: {visited}")
    print(f"Total estimated energy used: {round(total_energy, 4)} kWh")

    return full_path
