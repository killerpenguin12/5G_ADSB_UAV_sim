'''
File used for testing, not simulation 5/5/2020
'''
import sys
import numpy as np
import random
import propagation_param as p
from Vehicle import vehicle
import propagationFunctions as pf
# from flightPaths.openLogFiles import vehiclePositions, vehicleStates, vehicleEulers

# print('test 1', vehicleStates[0]['p'][:])
#
# for i in range(p.num_vehicles):
#     print('test 2', vehiclePositions[0][i])
# for i in range(1, 2*p.radius_of_area):
#     if pf.powerReceiveddBm(i) < -92.998 and pf.powerReceiveddBm(i) >= -93.0:
#         print(i)
# at 0.01 W, 3445 m is the max distance to hit MTL
# at 0.05 W, 7704 m
# at 0.1 W, 10896 m
# at 1 W, 34457 m

