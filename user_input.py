#!/usr/bin/env python3

def main():
    robot_battery_capacity = float(input("Please enter the Rover's battery capacity (kWh): "))
    robot_energy_consumption = float(input("Please enter rovers energy consumption per second (kWh): "))
    robot_charge_input = input("Please enter robot charge location (format: x,y or (x,y) or [x,y]): ")
    robot_charge_input = robot_charge_input.strip().replace("(", "").replace(")", "").replace("[", "").replace("]", "")
    robot_charge_locale = [int(x.strip()) for x in robot_charge_input.split(",")]

    time_to_wait = float(input("Please enter how long we must wait at a given data point (seconds): "))
    sensor_energy_consumption = float(input("Please enter energy consumed per second of scanning (kWh): "))
    sensor_range = float(input("Please enter the sensor range (meters): "))
    tile_size = float(input("Please enter the tile size (meters per grid cell): "))
    config_name = input("What would you like this configuration to be called?: ")
    map_name = input("What map would you like to map to? (.pgm): ")

    print("Please enter x,y coordinate of each data point, and 'B' when complete:")
    coord_list = []
    while True:
        coord = input()
        if coord.strip().upper() == "B":
            break
        coord = coord.strip().replace("(", "").replace(")", "").replace("[", "").replace("]", "")
        coord_split = coord.split(",")
        coord_tuple = (int(coord_split[0]), int(coord_split[1]))
        coord_list.append(coord_tuple)

    print("POIs collected:", coord_list)

    with open(f"{config_name}.yaml", "w") as file:
        file.write("# configuration for robot motion\n")
        file.write(f"map_name: {map_name}\n")
        file.write(f"tile_size: {tile_size}\n")
        file.write("robot:\n")
        file.write(f"  battery_capacity: {robot_battery_capacity}\n")
        file.write(f"  energy_consumption: {robot_energy_consumption}\n")
        file.write(f"  charge_locale: [{robot_charge_locale[0]}, {robot_charge_locale[1]}]\n")
        file.write("sensor:\n")
        file.write(f"  time_to_wait: {time_to_wait}\n")
        file.write(f"  energy_consuption: {sensor_energy_consumption}\n")
        file.write(f"  range: {sensor_range}\n")
        file.write("key_points:\n")
        for coord in coord_list:
            file.write(f"  - [{coord[0]}, {coord[1]}]\n")

if __name__ == "__main__":
    main()
