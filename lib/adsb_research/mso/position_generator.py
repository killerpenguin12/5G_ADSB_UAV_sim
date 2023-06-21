# Feb 1 2021
# This file writes positions for the simulations with random paths
#
#DO NOT USE THIS ONE, IT DOES NOT CHANGE ANYTHING. I HAVE NO IDEA WHY THERE ARE TWO FILES WITH THE SAME NAME. IT MAKES NO SENSE.
#
#
#
#
#
#
from xml.etree.ElementTree import PI
import numpy as np
import random
import csv
import sys, os
import math
sys.path.append(os.path.abspath(os.path.join('..', 'config')))
#import mso.propagation_param as p
from datetime import datetime
# from Vehicles.Vehicle import Vehicle
#random.seed(datetime.now())
random.seed(np.pi)

def generate_path(starting_position, ending_position, number_seconds, speed, positions, area_radius,
                  lower_altitude_lim, mid_altitude_lim, upper_altitude_lim):
    # print("Does the first run?")
    # print("#############################################1")
    # print("#############################################1")
    # print("#############################################1")
    n_distance = ending_position[0] - starting_position[0]
    e_distance = ending_position[1] - starting_position[1]
    d_distance = ending_position[2] - starting_position[2]
    total_distance = np.sqrt(n_distance ** 2. + e_distance ** 2. + d_distance ** 2.)
    num_steps = int(np.ceil(total_distance / speed))
    if num_steps < number_seconds:
        for i in range(num_steps):
            position = (starting_position[0] + (i * (n_distance / num_steps)),
                        starting_position[1] + (i * (e_distance / num_steps)),
                        starting_position[2] + (i * (d_distance / num_steps)))
            positions.append(position)
        if -ending_position[0] >= mid_altitude_lim:
            new_ending_altitude = random.randrange(mid_altitude_lim, upper_altitude_lim)
        else:
            new_ending_altitude = random.randrange(lower_altitude_lim, mid_altitude_lim)
        theta0 = random.random() * 2. * np.pi
        rand0 = np.sqrt(random.random())
        new_ending_position = (area_radius * rand0 * np.sin(theta0),
                               area_radius * rand0 * np.cos(theta0),
                               -new_ending_altitude)
        return generate_path(ending_position, new_ending_position, number_seconds - num_steps, speed, positions,
                             area_radius, lower_altitude_lim, mid_altitude_lim, upper_altitude_lim)
    else:
        for i in range(number_seconds):
            position = (starting_position[0] + (i * (n_distance / num_steps)),
                        starting_position[1] + (i * (e_distance / num_steps)),
                        starting_position[2] + (i * (d_distance / num_steps)))
            positions.append(position)
        
        return positions


def generate_random_paths(number_vehicles, number_seconds, area_radius, lower_speed_limit, upper_speed_limit,
                          lower_altitude_lim, mid_altitude_lim, upper_altitude_lim):
    all_positions = []
    # print("Does the second run?")
    # print("#############################################2")
    # print("#############################################2")
    # print("#############################################2")
    for i in range(number_vehicles):
        speed = random.randrange(lower_speed_limit, upper_speed_limit)
        if i % 2:
            beginning_altitude = random.randrange(mid_altitude_lim, upper_altitude_lim)
            ending_altitude = random.randrange(mid_altitude_lim, upper_altitude_lim)
        else:
            beginning_altitude = random.randrange(lower_altitude_lim, mid_altitude_lim)
            ending_altitude = random.randrange(lower_altitude_lim, mid_altitude_lim)
        theta0 = random.random() * 2. * np.pi
        rand0 = np.sqrt(random.random())
        start_position = (area_radius * rand0 * np.sin(theta0),
                          area_radius * rand0 * np.cos(theta0),
                          -beginning_altitude)
        theta1 = random.random() * 2. * np.pi
        rand1 = np.sqrt(random.random())
        end_position = (area_radius * rand1 * np.sin(theta1),
                        area_radius * rand1 * np.cos(theta1),
                        -ending_altitude)
        positions = []
        all_positions.append(generate_path(start_position, end_position, number_seconds, speed, positions, area_radius,
                                           lower_altitude_lim, mid_altitude_lim, upper_altitude_lim))
    # print(all_positions)
    return all_positions

# def generate_divided_paths(num_vehicles, num_seconds, area_radius, lower_speed_limit, upper_speed_limit,
#                           lower_altitude_lim, mid_altitude_lim, upper_altitude_lim):
#     all_positions = []
#     for i in range(num_vehicles):
#         speed = random.randrange(lower_speed_limit, upper_speed_limit)
#         """ 
#         Constraints:
#         Starting position must be a value that creates an LSB with %4=0
#         Ending position must be a value that creates an LSB with %4=0
#         """
#         if i % 2:
#             beginning_altitude = random.randrange(mid_altitude_lim, upper_altitude_lim)
#             ending_altitude = random.randrange(mid_altitude_lim, upper_altitude_lim)
#         else:
#             beginning_altitude = random.randrange(lower_altitude_lim, mid_altitude_lim)
#             ending_altitude = random.randrange(lower_altitude_lim, mid_altitude_lim)
        
#         vehicle = Vehicle((0,0,0), random.randrange(0,3600), 0)
#         start = False
#         end = False
#         startFinal = None
#         endFinal = None
#         while not start and not end:
#             theta0 = 2. * np.pi
#             theta1 = 2. * np.pi
#             if num_vehicles / (i+1) > 4:
#                 theta0 = random.random() * np.pi/2
#                 theta1 = random.random() * np.pi/2
#             elif num_vehicles / i > 2:
#                 theta0 = random.random() * np.pi/2 + np.pi/2
#                 theta1 = random.random() * np.pi/2 + np.pi/2
#             elif num_vehicles / i > 1.3333:
#                 theta0 = random.random() * np.pi/2 + np.pi
#                 theta1 = random.random() * np.pi/2 + np.pi
#             else:
#                 theta0 = random.random() * np.pi/2 + 3*np.pi/2
#                 theta1 = random.random() * np.pi/2 + 3*np.pi/2
            
#             rand0 = np.sqrt(random.random())
#             start_position = (area_radius * rand0 * np.sin(theta0),
#                             area_radius * rand0 * np.cos(theta0),
#                             -beginning_altitude)
#             rand1 = np.sqrt(random.random())
#             end_position = (area_radius * rand1 * np.sin(theta1),
#                             area_radius * rand1 * np.cos(theta1),
#                             -ending_altitude)
#             vehicleStart = vehicle.extract_lsbs(start_position)
#             vehicleEnd = vehicle.extract_lsbs(end_position)

#             if vehicleStart[0]%4 == 0 and vehicleEnd[0]%4 == 0:
#                 start = True
#                 startFinal = start_position
#             if vehicleEnd[0]%4 == 0 and vehicleEnd[0]%4 == 0:
#                 end = True
#                 endFinal = end_position
#         positions = []
#         all_positions.append(generate_path(startFinal, endFinal, num_seconds, speed, positions, area_radius,
#                                            lower_altitude_lim, mid_altitude_lim, upper_altitude_lim))
#     print(all_positions)
#     return all_positions
    


def generate_straight_path(number_seconds, start_position, end_position, positions):
    # print("Does the Third run?")
    # print("#############################################3")
    # print("#############################################3")
    # print("#############################################3")
    n_distance = end_position[0] - start_position[0]
    e_distance = end_position[1] - start_position[1]
    d_distance = end_position[2] - start_position[2]
    for i in range(number_seconds):
        position = (start_position[0] + (i * (n_distance / number_seconds)),
                    start_position[1] + (i * (e_distance / number_seconds)),
                    start_position[2] + (i * (d_distance / number_seconds)))
        positions.append(position)
    return positions

def generate_straight_path_even(number_seconds, start_position, end_position, positions):
    n_distance = end_position[0] - start_position[0]
    e_distance = end_position[1] - start_position[1]
    d_distance = end_position[2] - start_position[2]
    for i in range(number_seconds):
        position = (start_position[0] + (i * (n_distance / number_seconds)),
                    start_position[1] + (i * (e_distance / number_seconds)),
                    start_position[2] + (i * (d_distance / number_seconds)))
        positions.append(position)
    return positions

def twoPath(number_vehicles, number_seconds, area_radius, lower_altitude_lim, upper_altitude_lim):
    all_positions = []
    #print("Are we using this still?")
    for i in range(number_vehicles):
        if i%2 == 0: #even 
            start_position = (0,50)
            end_position = (0,0)
        else: #odd
            start_position = (0,-50)
            end_position = (0,0)
        positions = []
        all_positions.append(generate_straight_path(number_seconds, start_position, end_position, positions))
    return all_positions

def generate_collision_path(number_vehicles, number_seconds, area_radius, lower_altitude_lim, upper_altitude_lim):
    # print("#############################################5")
    # print("#############################################5")
    # print("Does this even run?")
    # print("#############################################5")
    # print("#############################################5")
    all_positions = []
    for i in range(number_vehicles):
        # if i % 2 == 0: #even 
        #     start_position = (0,0)
        #     end_position = (0,100)
        # else: #odd
        #     start_position = (0,100)
        #     end_position = (0,-100)
        pie = 3.14159265359
        start_position = (area_radius*math.cos(i*(4/pie)),area_radius*math.sin(i*(4/pie)),100)
        end_position = (area_radius*math.cos(i*(4/pie) + pie),area_radius*math.sin(i*(4/pie) + pie),100)
        # start_position = (area_radius*math.cos(i*(4/pie + pie),area_radius*math.sin(i*(4/pie) + pie)))
        # end_position = (area_radius*math.cos(i*(4/pie)),area_radius*math.sin(i*(4/pie)))
        positions = []
        # print("here!")
        # print(area_radius)
        all_positions.append(generate_straight_path(number_seconds, start_position, end_position, positions))
    
    # all_positions = []
    # current_angle = 0.0
    # angle_increment = np.pi / number_vehicles

    # for i in range(number_vehicles):
    #     random_param = 0.05
    #     beginning_altitude = random.randrange(lower_altitude_lim, upper_altitude_lim)
    #     ending_altitude = random.randrange(lower_altitude_lim, upper_altitude_lim)
    #     start_position = (area_radius * np.sin(current_angle +
    #                                            (random_param * random.random() * (1 if random.random() > 0.5 else -1))),
    #                       area_radius * np.cos(current_angle +
    #                                            (random_param * random.random() * (1 if random.random() > 0.5 else -1))),
    #                       -beginning_altitude)
    #     end_position = (area_radius * np.sin(current_angle + np.pi +
    #                                          (random_param * random.random() * (1 if random.random() > 0.5 else -1))),
    #                     area_radius * np.cos(current_angle + np.pi +
    #                                          (random_param * random.random() * (1 if random.random() > 0.5 else -1))),
    #                     -ending_altitude)
    #     positions = []
    #     all_positions.append(generate_straight_path(number_seconds, start_position, end_position, positions))

    #     if current_angle < 0:
    #         current_angle -= angle_increment
    #     else:
    #         current_angle += angle_increment
    #     current_angle = -current_angle
    return all_positions

    # generate_random_paths(p.num_vehicles, p.num_seconds, p.radius_of_area, p.lowest_speed, p.highest_speed,
    #                           p.lowest_altitude, p.middle_altitude, p.highest_altitude)
