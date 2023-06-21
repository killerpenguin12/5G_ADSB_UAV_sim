# Feb 1, 2021
# Basically analyzes the csv log files to determine
# the overall success rate of messages among all the vehicles

import numpy as np
from progress.bar import IncrementalBar

num_vehicles = 4  # this you have to just know
vehicle_names = [
                 # "PING20",
                 "TEST11",
                 "PING27",
                 # "PING3"
                 ]
vehicle_ICAO = ["FAA020",
                "AAAA11",
                "FAA027",
                "AAAAA3"
                ]
start_time = 54046  # you just have to know start and stop times.
stop_time = 54628
num_seconds = stop_time - start_time + 1

ownship_file_names = ["PING20_log_5_2020-07-23T15-00-43Z_Ownship_Messages.csv",
                      "PING20_log_6_2020-07-23T15-10-25Z_Ownship_Messages.csv",
                      # "PING25_log_5_2020-07-23T15-00-44Z_Ownship_Messages.csv",
                      "PING27_log_6_2020-07-23T15-00-43Z_Ownship_Messages.csv",
                      "PING27_log_7_2020-07-23T15-10-25Z_Ownship_Messages.csv",
                      # "PING30_log_5_2020-07-23T15-00-43Z_Ownship_Messages.csv",
                      # "PING30_log_6_2020-07-23T15-12-31Z_Ownship_Messages.csv"
                      ]
traffic_report_file_names = ["PING20_log_5_2020-07-23T15-00-43Z_Traffic_Reports.csv",
                             "PING20_log_6_2020-07-23T15-10-25Z_Traffic_Reports.csv",
                             # "PING25_log_5_2020-07-23T15-00-44Z_Traffic_Reports.csv",
                             "PING27_log_6_2020-07-23T15-00-43Z_Traffic_Reports.csv",
                             "PING27_log_7_2020-07-23T15-10-25Z_Traffic_Reports.csv",
                             # "PING30_log_5_2020-07-23T15-00-43Z_Traffic_Reports.csv",
                             # "PING30_log_6_2020-07-23T15-12-31Z_Traffic_Reports.csv"
                             ]

num_received = 0
num_desired = 0
bar = IncrementalBar("Simulating...", max=len(ownship_file_names))
for x in ownship_file_names:
    sending_vehicle = x[0:6]
    if sending_vehicle == "PING25":
        sending_vehicle = "TEST11"
    elif sending_vehicle == "PING30":
        sending_vehicle = "PING3"
    f = open(x, "r")
    for y in f:
        ownship_line = y.split(",")
        if ownship_line[0] == "time_stamp":
            continue
        second = int(np.floor(float(ownship_line[0])))
        # print(second)
        if start_time <= second <= stop_time:
            num_desired += 1
            for z in traffic_report_file_names:
                g = open(z, "r")
                for w in g:
                    report_line = w.split(",")
                    if report_line[0] == "time_stamp":
                        continue
                    received_second = int(np.floor(float(report_line[0])))
                    vehicle_name = report_line[20].replace(" ", "").replace("\"", "")
                    # print(z)
                    # print(second)
                    # print(received_second)
                    # print(vehicle_name)
                    # if received_second == second:
                    #     print(vehicle_name)
                    #     print(vehicle_name in vehicle_names)
                    #     for i in vehicle_names:
                    #         print("really?", i)
                    if received_second == second and vehicle_name == sending_vehicle:
                        num_received += 1
    bar.next()
bar.finish()
print(num_desired)
print(num_received)
print("Success rate: ", num_received/num_desired)
