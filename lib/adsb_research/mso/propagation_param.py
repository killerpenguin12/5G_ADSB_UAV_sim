# Feb 1, 2021
# Constants used for simulation. Also imports positional data.
# TODO split into separate files, including separating cases from positional data generation
import os

cwd = os.getcwd()  # Get the current working directory (cwd)
files = os.listdir(cwd)  # Get all the files in that directory
import numpy as np
import random
from math import radians, cos, sin, asin, sqrt
import position_generator as pg
import matplotlib.pyplot as plt
import pymap3d as pymap3d
from datetime import datetime

# Physical Constants
speed_of_light = 2.998 * 10.0 ** 8.0  # m/s, propagation velocity
# Frequency parameters for 978 MHz UAT
frequency = 978.0 * (10.0 ** 6.0)  # Hz, desired transmission frequency
wavelength = speed_of_light / frequency  # m, wavelength of 978 MHz
least_significant_bit = 360. / (2. ** 24.)  # degrees, conversion factor from DO-282B
def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r

'''
Now we import positional data
'''
# This section is for the direct collision course
if True:
    power_difference_decibel = 4.0
    receiver_min_trigger_level = -93.0  # dBm
    initial_lat = 40.2338  # degrees N, Provo
    # initial_lat = 45.1
    initial_lon = -111.6585  # degrees W, Provo
    # initial_lon = -121.1
    initial_altitude = 100.
    num_vehicles = 10
    num_seconds = 60 # s
    radius_of_area = 400  # m //not radius anymore, now its length of side for a square.
    lowest_speed = 1  # m/s 16
    highest_speed = 30  # m/s 51
    lowest_altitude = 99
    middle_altitude = 100
    highest_altitude = 101
    num_desired_decodes = num_seconds * num_vehicles * (num_vehicles - 1)
    effective_radiated_power = 10.0

    #seed = np.pi  # is the original. then 42
    #random.seed(np.pi)#
    random.seed(datetime.now())
    vehicle_positions = pg.generate_random_paths(num_vehicles, num_seconds, radius_of_area, lowest_speed, highest_speed,
                                               lowest_altitude, middle_altitude, highest_altitude)
    #print("Do we get here?")
    #vehicle_positions = pg.generate_collision_path(num_vehicles, num_seconds, radius_of_area, lowest_altitude,
    #                                                highest_altitude)
    #print(vehicle_positions[0])
    # vehicle_positions = pg.twoPath2(num_vehicles, num_seconds, radius_of_area, lowest_altitude,
    #                                               highest_altitude)
    # vehicle_positions = pg.twoPath(num_vehicles, num_seconds, radius_of_area, lowest_altitude,
    #                                               highest_altitude)
    #vehicle_positions = pg.retrieve_path(num_vehicles,num_seconds)
    #print("Length: ",len(vehicle_positions[21]))
    #print(vehicle_positions)

# This section is for the direct collision course. This refers to vehicles spaced around a circle that then fly across
# the circle to get to the other side. In the middle, their Lat/Lon positions somewhat converge, though they should be
# altitude deconflicted, and noise in the coordinates means they don't have exactly the same position.
elif False:
    coordinates_NED = True
    num_vehicles = 10
    num_seconds = 325  # s
    radius_of_area = 3000  # m
    lowest_altitude = 100
    highest_altitude = 151
    num_desired_decodes = num_seconds * num_vehicles * (num_vehicles - 1)

    seed = np.pi  # is the original. then 42
    random.seed(seed)
    vehicle_positions = pg.generate_collision_path(num_vehicles, num_seconds, radius_of_area, lowest_altitude,
                                                   highest_altitude)

    if False:  # if true, plots all paths with circle as well. There is a better plot in the main_sim file commented out
        plt.rcParams.update({'font.size': 15})
        fig = plt.figure()
        # plt.gcf().subplots_adjust(left=0.15)
        circle = plt.Circle((0, 0), radius_of_area, edgecolor='k', facecolor='none')

        ax = fig.add_subplot(111)
        plt.gca().add_artist(circle)

        for i in range(num_vehicles):
            north_vals = []
            east_vals = []
            for j in range(num_seconds):
                north_vals.append(vehicle_positions[i][j][0])
                east_vals.append(vehicle_positions[i][j][1])
            ax.scatter(north_vals, east_vals, s=1)

        ax.set_aspect('equal')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        plt.show()




