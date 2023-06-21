# Feb 1, 2021
# I believe I used this file to check some of Tanner's math that appears in the
# SciTech 2021 conference paper.

import propagation_param as p
import numpy as np
from dec2bin import dec2bin
import pymap3d as pymap3d


binary_value = p.initial_lat
if binary_value < 0:
    binary_value += 180  # negative values (South for LAT, West for LON)
    # have slightly different encoding than positive
binary_value = binary_value / p.least_significant_bit  # LSB is a conversion factor in D0-282B. 360/(2^24)
binary_value = np.round_(binary_value)  # round to the nearest integer to prep for binary conversion
binary_value = np.int(binary_value)  # change to an int for binary conversion, otherwise the format is weird
first1 = binary_value
print(first1)
binary_value = dec2bin(binary_value)  # converts to a binary string
first = first1 + 1
print(first)
second = dec2bin(first)
print(binary_value)
print(second)

original = first1 * p.least_significant_bit
new = first * p.least_significant_bit
print(original)
print(new)

phi1 = original
phi2 = new
delta_phi = phi2 - phi1
lambda1 = p.initial_lon
lambda2 = p.initial_lon
delta_lambda = lambda2 - lambda1
R = 6378100
a = (np.sin((delta_phi * np.pi/180.)/2.))**2.0 + np.cos(phi1*np.pi/180.) * np.cos(phi2*np.pi/180.) * (np.sin((delta_lambda * np.pi/180.)/2.))**2.0
c = 2. * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
d = R * c
print(d)

ned_original = pymap3d.geodetic2ned(original, p.initial_lon, p.initial_altitude, p.initial_lat, p.initial_lon, p.initial_altitude)
ned_new = pymap3d.geodetic2ned(new, p.initial_lon, p.initial_altitude, p.initial_lat, p.initial_lon, p.initial_altitude)

print(ned_original)
print(ned_new)
print(ned_new[0] - ned_original[0])
### Tanner's math on the conversion checks out!
# print(binary_value)
# binary_value_12 = binary_value[(len(binary_value) - 12):len(binary_value)]  # discards all but the last 12 bits
# binary_value_12 = int(binary_value_12, 2)  # turns the 12 bit binary string into an integer value
# binary_value_12 += 1
# binary_value_12 = dec2bin(binary_value_12)
# new_value = []
# for i in range(12):
#     new_value.append(binary_value[i])
#
# for j in range(12):
#     new_value.append(binary_value_12[i])
# # return binary_value  # returns the correctly encoded 12 L.S.B.s of the value
