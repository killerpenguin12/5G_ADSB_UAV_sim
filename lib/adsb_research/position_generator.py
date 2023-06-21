# Feb 1 2021
# This file writes positions for the simulations with random paths

from xml.etree.ElementTree import PI
import numpy as np
import random
import csv
import sys, os
import pandas as pd
from math import sin,cos,pi
sys.path.append(os.path.abspath(os.path.join('..', 'config')))
# import mso.propagation_param as p
from datetime import datetime
# from Vehicles.Vehicle import Vehicle
random.seed(datetime.now())
#random.seed(5)

def generate_path(starting_position, ending_position, number_seconds, speed, positions, area_radius,
                  lower_altitude_lim, mid_altitude_lim, upper_altitude_lim):
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

def retrieve_path(number_vehicles,number_seconds):
    #This is getting our Positions from a provided CSV
    #organized in a wrong way, currently I am getting the same start positions for
    #each vehicle, need to vary it between vehicles. So it reads it in as:
    full_position = []
    positions = []
    firstread = pd.read_csv('Positions.csv',header=None)
    firstread.columns = ["id","x","y"]
    count = 100
    for j in range(number_vehicles):
        #for i in range(number_seconds):
        one = firstread.loc[firstread['id'] == j]
        x = one['x'].values.tolist()
        y = one['y'].values.tolist()
        z = list(np.ones_like(x) * 100)
        count += 1
        positions = []
        #if(count >= 100):
        for k in range(number_seconds):
            #print("k: ",k)
            position = (y[k],x[k],z[k])
            positions.append(position)
        count = 0
        #print(len(positions))
        full_position.append(positions)

    return full_position



def generate_random_paths(number_vehicles, number_seconds, area_radius, lower_speed_limit, upper_speed_limit,
                          lower_altitude_lim, mid_altitude_lim, upper_altitude_lim):
    all_positions = []
    for i in range(number_vehicles):
        speed = random.randrange(lower_speed_limit, upper_speed_limit)
        if i % 2:
            beginning_altitude = random.randrange(mid_altitude_lim, upper_altitude_lim)
            ending_altitude = random.randrange(mid_altitude_lim, upper_altitude_lim)
        else:
            beginning_altitude = random.randrange(lower_altitude_lim, mid_altitude_lim)
            ending_altitude = random.randrange(lower_altitude_lim, mid_altitude_lim)
        
        theta0 = random.random() * 2. * np.pi
        #theta0 = random.randrange(-np.pi,np.pi)
        #print("Theta0: ",theta0)
        rand0 = np.sqrt(random.random())
        rand2 = random.random() #used to get a random number between 0 and 1
        rand3 = random.random()
        #start_position = (area_radius * rand0 * np.sin(theta0),
        #                  area_radius * rand0 * np.cos(theta0),
        #                  -beginning_altitude)
        start_position = ((area_radius) * np.sin(theta0),
                          (area_radius) * np.cos(theta0),
                          -beginning_altitude)
        theta1 = random.random() * 2. * np.pi
        #theta1 = random.randrange(-np.pi,np.pi)
        #print("Theta1: ",theta1)
        rand1 = np.sqrt(random.random())
        end_position = ((area_radius) * np.sin(theta1),
                        (area_radius) * np.cos(theta1),
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
    beginning_altitude = random.randrange(lower_altitude_lim, upper_altitude_lim)
    ending_altitude = random.randrange(lower_altitude_lim, upper_altitude_lim)
    for i in range(number_vehicles):
        # if i%2 == 0: #even 
        #     start_position = (0,100,beginning_altitude)
        #     end_position = (0,0,ending_altitude)
        # else: #odd
        #     start_position = (0,-100,beginning_altitude)
        #     end_position = (0,0,ending_altitude)
        start_position = (i*2,area_radius,beginning_altitude)
        end_position = (i*2,-area_radius,beginning_altitude)
        positions = []
        all_positions.append(generate_straight_path(number_seconds, start_position, end_position, positions))
    return all_positions

def twoPath2(number_vehicles, number_seconds, area_radius, lower_altitude_lim, upper_altitude_lim):
    all_positions = []
    beginning_altitude = random.randrange(lower_altitude_lim, upper_altitude_lim)
    ending_altitude = random.randrange(lower_altitude_lim, upper_altitude_lim)
    #st_pos_x = [80.3821,94.9373,19.0123,43.7785,-51.6948,97.6034,-27.5428,-98.4821,-84.5144,-42.6107,-43.169,-95.8829,-48.5004,56.2464,99.7957,-73.9729,-99.479,-7.68617,13.8949,72.1798]
    #st_pos_y = [59.4871,-31.4152,98.176,-89.908,-85.6017,-21.7617,-96.1322,17.357,-53.4538,90.4673,90.2022,-28.3986,-87.4512,-82.6822,-6.38868,67.2905,10.1948,-99.7042,99.0299,69.2104]
    #end_pos_x = [66.4727,78.5094,15.7223,36.2028,-42.7493,80.7142,-22.7767,-81.4409,-69.89,-69.3013,-35.6988,-79.2913,40.1077,46.5133,82.5272,-61.1725,-82.2653,-6.35611,11.4905,59.6897]
    #end_pos_y = [49.1938,-25.9794,81.1878,-74.3506,-70.7895,-17.9962,-79.4976,14.3537,-44.2045,84.9238,74.5939,-23.4847,-72.3189,-68.3752,-5.28323,55.6469,8.43077,-82.4515,81.894,57.2346]
    
    st_pos_x = [-91.8148,-31.8789,15.4906,96.6545,73.2161,-23.4225,-54.2696,-53.2498,-52.3273,-90.9359,95.8446,56.6678,-97.0612,-99.9983,99.439,82.5386,74.1769,-8.42165,-1.44534,-24.9721]
    st_pos_y = [-39.6238,94.7826,98.7929,25.6499,68.1132,-97.2182,-83.9929,-84.6431,85.2165,-41.6012,28.5275,82.394,-24.0648,-0.579161,-10.5779,56.4569,67.0656,-99.6447,99.9896,96.8318]
    end_pos_x = [-75.9272,-26.3624,12.81,79.9295,60.5466,-19.3694,-44.8786,-44.0353,-43.2724,-75.2004,79.2597,46.8618,-80.2659,-82.6948,82.2322,68.256,61.3412,-6.96432,-1.19523,-20.6508]
    end_pos_y = [-32.7676,78.3816,81.6979,21.2116,56.3272,-80.3958,-69.459,-69.9967,70.4709,-34.4028,23.5914,68.1368,-19.9008,-0.478948,-8.74758,46.688,55.4609,-82.4024,82.6875,80.0762]
    # print("1: ",len(st_pos_x))
    # print("2: ",len(st_pos_y))
    # print("3: ",len(end_pos_x))
    # print("4: ",len(end_pos_y))
    for i in range(number_vehicles):
        start_position = (st_pos_x[i],st_pos_y[i],beginning_altitude)
        end_position = (end_pos_x[i],end_pos_y[i],ending_altitude)
        positions = []
        all_positions.append(generate_straight_path(number_seconds, start_position, end_position, positions))
    #START
    #x: -91.8148,-31.8789,15.4906,96.6545,73.2161,-23.4225,-54.2696,-53.2498,-52.3273,-90.9359,95.8446,56.6678,-97.0612,-99.9983,99.439,82.5386,74.1769,-8.42165,-1.44534,-24.9721
    #y: -39.6238,94.7826,98.7929,25.6499,68.1132,-97.2182,-83.9929,-84.6431,85.2165,-41.6012,28.5275,82.394,-24.0648,-0.579161,-10.5779,56.4569,67.0656,-99.6447,99.9896,96.8318

    
    #END
    #x: -75.9272,-26.3624,12.81,79.9295,60.5466,-19.3694,-44.8786,-44.0353,-43.2724,-75.2004,79.2597,46.8618,-80.2659,-82.6948,82.2322,68.256,61.3412,-6.96432,-1.19523,-20.6508

    #y: -32.7676,78.3816,81.6979,21.2116,56.3272,-80.3958,-69.459,-69.9967,70.4709,-34.4028,23.5914,68.1368,-19.9008,-0.478948,-8.74758,46.688,55.4609,-82.4024,82.6875,80.0762


    return all_positions

def generate_collision_path(number_vehicles, number_seconds, area_radius, lower_altitude_lim, upper_altitude_lim):
    all_positions = []
    current_angle = 0.0
    angle_increment = np.pi / number_vehicles

    for i in range(number_vehicles):
        random_param = 0.05
        beginning_altitude = random.randrange(lower_altitude_lim, upper_altitude_lim)
        ending_altitude = random.randrange(lower_altitude_lim, upper_altitude_lim)
        # start_position = (area_radius * np.sin(current_angle +
        #                                        (random_param * random.random() * (1 if random.random() > 0.5 else -1))),
        #                   area_radius * np.cos(current_angle +
        #                                        (random_param * random.random() * (1 if random.random() > 0.5 else -1))),
        #                   beginning_altitude)
        # end_position = (area_radius * np.sin(current_angle + np.pi +
        #                                      (random_param * random.random() * (1 if random.random() > 0.5 else -1))),
        #                 area_radius * np.cos(current_angle + np.pi +
        #                                      (random_param * random.random() * (1 if random.random() > 0.5 else -1))),
        #                 ending_altitude)
        start_position = (area_radius * sin(i*(4/pi)),area_radius * cos(i*(4/pi)),beginning_altitude)
        end_position = (area_radius * sin(i*(4/pi) + pi),area_radius * cos(i*(4/pi) + pi),beginning_altitude)
        positions = []
        all_positions.append(generate_straight_path(number_seconds, start_position, end_position, positions))

        if current_angle < 0:
            current_angle -= angle_increment
        else:
            current_angle += angle_increment
        current_angle = -current_angle
    return all_positions

# generate_random_paths(p.num_vehicles, p.num_seconds, p.radius_of_area, p.lowest_speed, p.highest_speed,
#                           p.lowest_altitude, p.middle_altitude, p.highest_altitude)
