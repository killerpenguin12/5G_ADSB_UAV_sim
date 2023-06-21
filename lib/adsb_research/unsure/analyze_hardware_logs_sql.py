# Feb 19, 2021
# Basically analyzes the sql log files to determine
# the overall success rate of messages among all the vehicles
import os
from progress.bar import IncrementalBar

start_time = 54046  # you just have to know start and stop times.
stop_time = 54628
file_path = "/home/jonathan/Research/adsb_research/hardware_sql_logs/"  # Path to folder containing sql files
file_names = os.listdir(file_path)
file_names[:] = [x for x in file_names if "Static" not in x and "Status" not in x]  # Discard irrelevant files
ownship_file_names = []
traffic_file_names = []

icao_set = set([])
vehicle_names_set = set([])
name_index = 0
icao_index = 4
for f in file_names:
    if "Ownship" in f:
        ownship_file_names.append(f)
        vehicle_name = f.split("_")[name_index]
        vehicle_names_set.add(vehicle_name)
    elif "Traffic" in f:
        traffic_file_names.append(f)
        f = open(file_path + f, "r")
        for line in f:
            icao = line.split(",")[icao_index].replace("\"", "")
            icao_set.add(icao)

vehicle_map = {}
for name in vehicle_names_set:
    for icao in icao_set:
        if icao[-2:] in name:
            vehicle_map[icao] = name
            break

timestamp_index = 2
num_received_messages = 0
num_desired_messages = 0
progress_bar = IncrementalBar("Simulating...", max=len(ownship_file_names))
for x in ownship_file_names:
    f = open(file_path + x, "r")
    sending_vehicle = x.split("_")[name_index]
    for y in f:
        ownship_line = y.split(",")
        transmitted_second = int(ownship_line[timestamp_index])
        if start_time <= transmitted_second <= stop_time:
            num_desired_messages += 1
            for z in traffic_file_names:
                g = open(file_path + z, "r")
                for w in g:
                    traffic_line = w.split(",")
                    received_second = int(traffic_line[timestamp_index])
                    vehicle_name = vehicle_map[traffic_line[icao_index].replace("\"", "")]
                    if received_second == transmitted_second and vehicle_name == sending_vehicle:
                        num_received_messages += 1
    progress_bar.next()
progress_bar.finish()
print(num_desired_messages)
print(num_received_messages)
print("Success rate: ", num_received_messages/num_desired_messages)
