'''
No longer used as of 5/5/2020 (this code runs in the propagation_param.py file)
Parameters used in simulation. Also imports positional data from .log files
'''
# Based off of files provided by Jaron Ellingson
import os
import sys
sys.path.append("..")
import numpy as np
import mso.propagation_param as p

'''
The two variables below should be verified before simulation
'''
folderName = 'fifteenVehicles'
number_vehicles = 10
sample_rate = 1000

sys.path.append(folderName)

def load_positions():
    return vehiclePositions

stateType = np.dtype([('t',np.float64),
    ('p',np.float64,(3,1)),
    ('v',np.float64,(3,1)),
    ('lin_accel',np.float64,(3,1)),
    ('q',np.float64,(4,1)),
    ('omega',np.float64,(3,1)),
    ('ang_accel',np.float64,(3,1))])

eulerType = np.dtype([('t',np.float64),
    ('ang',np.float64,(3,1))])

vehicleStates = []
vehicleEulers = []
vehiclePositions = []

for i in range(p.num_vehicles):
    vehicleStates.append(np.fromfile('/tmp/quad'+str(i)+'_true_state.log', dtype=stateType))
    # vehicleEulers.append(np.fromfile('flightPaths/'+folderName+'/quad'+str(i)+'_euler_angles.log', dtype=eulerType))
    # vehicleStates.append(np.fromfile('/tmp/quad'+str(i)+'_true_state.log', dtype=stateType))
    # vehicleEulers.append(np.fromfile('/tmp/quad'+str(i)+'_euler_angles.log', dtype=eulerType))

number_seconds = int(np.floor(len(vehicleStates[0]['t'])//sample_rate))

if(p.num_vehicles != len(vehicleStates)):
    txt = "The following don't match: \np.num_vehicles: {} \nVehicles in data: {}"
    print(txt.format(p.num_vehicles, len(vehicleStates)))
if(p.num_seconds != np.floor(len(vehicleStates[0]['t'])/sample_rate)):
    txt = "The following don't match: \np.num_seconds: {} \nSeconds in data: {}"
    print(txt.format(p.num_seconds, np.floor(len(vehicleStates[0]['t'])/sample_rate)))
# print(vehicleStates[0]['p'])
#print(vehicleStates[0]['t'])

#vehicleStates[vehicle]['desired_thing'][second/frame][index within tuple]

# for i in range(p.num_vehicles):
#     positions = []
#     for j in range(len(vehicleStates[i])):
#         position = vehicleStates[i]['p'][j]
#         positions.append(position)
#     vehiclePositions.append(positions)
# print(len(vehicleStates[i]['p'][0 : number_seconds * sample_rate : sample_rate]))
for i in range(p.num_vehicles):
    positions = vehicleStates[i]['p'][0 : number_seconds * sample_rate : sample_rate].copy()
    vehiclePositions.append(positions)
# for i in range(p.num_vehicles):
#     print("Height:", vehicleStates[i][0]['p'][2])

# for i in range(p.num_seconds):
#     print("V-0 height:", vehicleStates[1]['p'][i][0])

#print(vehicleStates)
