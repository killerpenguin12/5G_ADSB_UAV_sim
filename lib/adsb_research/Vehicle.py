
# This file is the vehicle class used in normal simulation based on non gps tracks

import numpy as np
import pymap3d as pymap3d
import mso.propagation_param as p
from dec2bin import dec2bin
import csv


class Vehicle:
    def __init__(self, position, frame=0, vehicle_identifier=0):
        self.position = 0
        self.latitude_lsbs = 0  # 12 L.S.B.'s of the most recent valid "LATITUDE"
        self.longitude_lsbs = 0  # 12 L.S.B.'s of the most recent valid "LONGITUDE"
        self.update_position(position)
        self.frame = frame  # current UAT frame
        self.previous_random_number = 0  # R(m-1) most recent random number chosen
        self.frame_before_restricted = 0  # the frame just prior to entering the restricted MSO range mode
        self.message_start_opportunity = 0  # current message start opportunity
        self.vehicle_identifier = vehicle_identifier
        self.transmissions = []

    # Returns the 12 L.S.B.'s of the most recent valid "LATITUDE" or
    # "LONGITUDE", based on the value of m
    # DO-282B pg. 84-85
    def get_least_significant_bits(self, frame):
        if frame == 0:
            #print("LSB 0")
            return self.latitude_lsbs  # latitude
        elif frame == 1:
            #print("LSB 1")
            return self.longitude_lsbs  # longitude
        else:
            print("ERROR: frame is invalid in get_least_significant_bits")

    # Returns a pseudo-random number
    # (In order to get R(m -1), this function cannot be used,
    # since R(m - 1) is the previously chosen random number.
    # If the vehicle latitude or longitude has changed, this
    # function may return a different value than the previous
    # random number) DO-282B pg. 84-85
    def pseudo_random_number(self, frame):
        if frame == 0:
            self.previous_random_number = self.get_least_significant_bits(0) % 3200  # calculate pseudo-random number
            return self.previous_random_number
        elif frame >= 1:
            # calculate pseudo-random number
            self.previous_random_number = (4001 * self.previous_random_number +
                                           self.get_least_significant_bits(frame % 2)) % 3200
            return self.previous_random_number
        else:
            print("ERROR: frame is invalid in pseudo_random_number")

    # See MSO details on pdf page 115 of of DO-B282 (page 85)
    # Returns the initial MSO when it is first being calculated
    # DO-282B pg. 84-85
    def full_mso_range(self):
        self.frame_before_restricted = self.frame  # update k
        # print("VEHICLE PRINTING")
        # print("##################################################")
        # print("Frame: ",self.frame)
        # print("Position: ",self.position)
        # print("Random Number: ",self.pseudo_random_number(self.frame))
        self.message_start_opportunity = (752 + self.pseudo_random_number(self.frame))  # calculate MSO
        # print("Frame: ", self.frame)
        # print("Random Number: ", self.pseudo_random_number(self.frame))
        # print("mso: ",self.message_start_opportunity)
        # print("lat: ",self.latitude_lsbs)
        # print("long: ",self.longitude_lsbs)
        # print("END OF VEHICLE PRINTING")
        # with open('Jonathan_LSBS_test.csv', 'a', newline='') as csvfile: #after first run change to a
        #     writer = csv.writer(csvfile, delimiter=',')            
        #     #writer.writerow(["Num Vehicles","Watts","Total MSO","Area Radius","overlap","chance","closeness","collisions","MTL","High Power","seed"])
        #     writer.writerow([str(self.latitude_lsbs),str(self.longitude_lsbs),str(self.frame),str(self.position[0]),
        #     str(self.position[1]),str(self.message_start_opportunity)])
        # # print("##################################################")
        # print(" ")
        return self.message_start_opportunity  # return MSO

    # Returns an MSO once the vehicle is no longer in the full MSO range mode
    # DO-282B pg. 84-85
    def restricted_mso_range(self):
        r_star = self.pseudo_random_number(self.frame_before_restricted) - (
                self.pseudo_random_number(self.frame_before_restricted) % 800)  # definition of R_star (R*)
        self.message_start_opportunity = (752 + r_star + (self.pseudo_random_number(self.frame) % 800))  # calculate MSO
        return self.message_start_opportunity  # return MSO

    # Converts NED coordinate to geodetic, based on a relative geodetic coordinate
    @staticmethod
    def get_lat_lon_alt(position):
        #print("Position for digits: ", position[0], " ",position[1])
        return pymap3d.ned2geodetic(position[0], position[1], position[2], p.initial_lat, p.initial_lon,
                                    p.initial_altitude, pymap3d.Ellipsoid("wgs84"))

    # This is the encoding set forth in DO-282B
    @staticmethod
    def encode_lsbs(value):
        binary_value = value
        if binary_value < 0:
            binary_value += 180  # negative values (South for LAT, West for LON)
            # have slightly different encoding than positive
        binary_value = binary_value / p.least_significant_bit  # LSB is a conversion factor in D0-282B. 360/(2^24)
        binary_value = np.round_(binary_value)  # round to the nearest integer to prep for binary conversion
        binary_value = np.int(binary_value)  # change to an int for binary conversion, otherwise the format is weird
        binary_value = dec2bin(binary_value)  # converts to a binary string
        #print("Before: ",binary_value)
        binary_value = binary_value[(len(binary_value) - 12):len(binary_value)]  # discards all but the last 12 bits
        binary_value = binary_value[0:12]
        #print("After: ",binary_value)
        binary_value = int(binary_value, 2)  # turns the 12 bit binary string into an integer value
        #print("Here is the final value: ",binary_value)
        return binary_value  # returns the correctly encoded 12 L.S.B.s of the value

    def extract_lsbs(self, position):
        coordinate = self.get_lat_lon_alt(position)
        latitude_lsbs = self.encode_lsbs(coordinate[0])
        longitude_lsbs = self.encode_lsbs(coordinate[1])
        # print("VEHICLE PRINTING")
        # #print("Type: ",np.dtype(position))
        # #print("Position: ",position)
        # print("Coord0: ",coordinate[0])
        # print("Coord1: ",coordinate[1])
        # print("latitde_lsbs: ",latitude_lsbs)
        # print("Long_lsbs: ",longitude_lsbs)
        # print(" ")
        return latitude_lsbs, longitude_lsbs, coordinate[2]

    # This updates the 12 L.S.B.s of the latitude and longitude
    def update_position(self, new_position):
        self.position = new_position
        #print("VEHICLE PRINTING")
        # print("Position: ",self.position)
        self.latitude_lsbs, self.longitude_lsbs, altitude = self.extract_lsbs(new_position) #backwards from regular
        #print("lat: ",self.latitude_lsbs)
        #print("long: ",self.longitude_lsbs)
        #print("END OF VEHICLE PRINTING")