#!/usr/bin/env python3



def main():
    robot_battery_capacity = float(input("Please enter the Rover's battery capacity (kWh): "))
    robot_energy_consumption = float(input("Please enter rovers energy consumption per second (kWh): "))
    robot_charge_locale = input("Please enter robot charge location: ")
    time_to_wait = float(input("Please enter how long we must wait at a given data point (seconds): "))
    sensor_energy_consumption = float(input("Please enter energy consumed per second of scanning (kWh): "))
    sensor_range = float(input("Please enter the sensor range (meters): "))
    config_name = input("What would you like this configuration to be called?: ")
    map_name = input("What map would you like to map to? (.pgm): ")
    not_done = True
    print("Please enter x,y coordinate of each data point, and 'B' when complete:")
    coord_list = []
    while not_done:
        coord = input()
        if coord == "B":
            not_done = False
            break
        coord_split = coord.split(",")
        coord = (int(coord_split[0]), int(coord_split[1]))
        coord_list.append(coord)
    print(coord_list)

    with open(f"{config_name}.yaml", "w") as file:
        file.write("# configuration for robot motion")
        file.write(f"\nmap_name: {map_name}\n")
        file.write(f"robot:\n")
        file.write(f"  battery_capacity: {robot_battery_capacity}\n")
        file.write(f"  energy_consumption: {robot_energy_consumption}\n")
        file.write(f"sensor:\n")
        file.write(f"  time_to_wait: {time_to_wait}\n")
        file.write(f"  energy_consuption: {sensor_energy_consumption}\n")
        file.write(f"  range: {sensor_range}\n")
        file.write(f"key_points:\n")
        for coord in coord_list:
            file.write(f"  - {coord}\n")
        
    





if __name__ == "__main__":
    main()