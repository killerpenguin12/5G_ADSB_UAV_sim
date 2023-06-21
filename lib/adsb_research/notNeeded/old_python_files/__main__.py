'''
Most current main file. 5/5/2020
'''
import sys
import numpy as np
import propagation_param as p
from Vehicle import vehicle
import propagationFunctions as pf
import random
import matplotlib.pyplot as plt
import mplcursors
# from flightPaths.openLogFiles import vehiclePositions, vehicleStates, vehicleEulers
from myStructs import MSO_collision, transmission
from itertools import groupby
from operator import itemgetter

'''
To run the code:
- Install all of the necessary packages
- Verify the correct folderName and num_vehicles in
  propagation_param.py
'''
# def main():
#     print("boom")
#
# if __name__ == "__main__":
#     main()

vehicles = []  # Declare list of vehicles, empty for now
positions = []
MSOs = []  # Will store every MSO calculated, so we can look at the frequency of each MSO
seconds = []  # List that will have a list of MSOs for every second, to organize them better
MSO_collisions = []  # Will record any MSO that is calculated by two or more vehicles in the same second
vehicles_per_collision = []  # Will keep track of how many __-way MSO collisions there are
unsuccessful_decodes = 0
successful_decodes = 0
false_from_closeness = 0
false_from_collision = 0
false_from_message_overlap = 0
false_from_MTL = 0
false_from_high_power = 0

for i in range(p.num_vehicles):
    myPositions = p.vehiclePositions[i][:]
    positions.append(myPositions)

    # Populate vehicles list with vehicles
    uas = vehicle(positions[i][0], 0, i)  # FIXME indexing
    vehicles.append(uas)
# print(vehicles[0].binaryEncodeLSBs(p.lat0))
# The rest of the time we do the other MSO calculation
for i in range(p.num_seconds):
    if i % p.one_percent == 0 and i:
        txt = "{}% through the main loop"
        percentage = i / p.one_percent
        print(txt.format(percentage))
    # Puts a list of MSOs in for every second in seconds array
    MSOs_per_second = []
    seconds.append(MSOs_per_second)
    collision_objects = []
    # Before calculating MSOs, update the vehicle positions
    if i > 0:  # dont update for first second
        for j in range(p.num_vehicles):
            vehicles[j].updatePosition(positions[j][i])

    # Begin MSO calculations
    for j in range(p.num_vehicles):
        MSO = vehicles[j].fullMSORange()  # calculate MSO
        myTransmissions = []
        for k in range(p.num_vehicles):
            myTransmissions.append(0)
        # print("initial k: ", vehicles[j].vehicle_identifier + 1)
        # final_k = 0
        for k in range(vehicles[j].vehicle_identifier + 1, vehicles[j].vehicle_identifier + p.num_vehicles):
            final_k = k
            power_dBm = pf.combinedReceivedPower(vehicles[j], vehicles[k % p.num_vehicles])
            # print(power_dBm)
            # print("then corresponding power: ",power_dBm)
            meets_MTL = True
            if power_dBm > -3.0:
                meets_MTL = False
                false_from_high_power += 1
            elif power_dBm >= p.receiver_min_trigger_level:
                meets_MTL = True
            else:
                meets_MTL = False
                false_from_MTL += 1
            newTransmission = transmission(i, MSO, vehicles[j].vehicle_identifier, \
                                           vehicles[k % p.num_vehicles].vehicle_identifier, \
                                           pf.distanceBetweenVehicles(vehicles[j], vehicles[k % p.num_vehicles]), \
                                           power_dBm, vehicles[j].m, meets_MTL)
            myTransmissions[newTransmission.receiver_ID] = newTransmission
            '''if power_dBm >= p.receiver_min_trigger_level else False'''
        # print("final k: ", final_k)
        # print(len(myTransmissions))
        vehicles[j].transmissions.append(myTransmissions)
        vehicles[j].m += 1  # increase frame
        MSOs.append(MSO)  # record MSO
        seconds[i].append(MSO)  # record MSO in a way that helps with collisions

    for j in range(p.num_vehicles):
        for k in range(p.num_vehicles):  # this counts failing because you're switching modes. It also counts failure to decode because you're on the same MSO
            if vehicles[j].transmissions[i][k] == 0:
                continue
            if ((vehicles[j].MSO >= (vehicles[vehicles[j].transmissions[i][k].receiver_ID].MSO - 8)) and (
                    vehicles[j].MSO <= (vehicles[vehicles[j].transmissions[i][k].receiver_ID].MSO + 8))):
                if vehicles[j].transmissions[i][k].successful_decode:
                    vehicles[j].transmissions[i][k].successful_decode = False
                    false_from_closeness += 1

    for j in range(752, 3952):  # for each MSO
        count = seconds[i].count(j)  # count how many times that MSO occurred in the second
        if count > 1:  # if the MSO occurred more than once, there was an MSO collision
            # do something to figure out which vehicles
            vehicles_involved = []  # records which vehicles are involved in the collision
            for k in range(p.num_vehicles):  # for each vehicle
                if seconds[i][k] == j:  # if the MSO matches the one of the collision
                    vehicles_involved.append(
                        k)  # record the vehicle number, which corresponds to an index in the vehicles list
            vehicles_per_collision.append(count)  # Record how many vehicles per collision
            MSO_collisions.append(j)  # record MSO where collision happened
            new_collision = MSO_collision(i, j, vehicles_involved)  # record the collision as object
            collision_objects.append(new_collision)  # add the new collision to the list

    sorted_MSOs = seconds[i].copy()
    sorted_MSOs.sort()
    consecutive_MSOs = []
    for k, g in groupby(enumerate(sorted_MSOs), lambda ix: ix[0] - ix[1]):
        my_consecutives = list(map(itemgetter(1), g)).copy()
        if len(my_consecutives) > 1:
            consecutive_MSOs.append(my_consecutives)

    # if(len(consecutive_MSOs) != 0):
    #     print(consecutive_MSOs)
    for j in range(len(consecutive_MSOs)):
        for k in range(len(consecutive_MSOs[j])):
            for l in range(p.num_vehicles):
                if vehicles[l].MSO == consecutive_MSOs[j][k] and ((k % 2) != 0):
                    for m in range(p.num_vehicles):
                        if vehicles[l].transmissions[i][m] != 0:
                            if vehicles[l].transmissions[i][m].successful_decode:
                                vehicles[l].transmissions[i][m].successful_decode = False
                                false_from_message_overlap += 1
    # consecutive_MSOs = []
    # print(sorted_MSOs)
    # for j in range(len(sorted_MSOs) - 1):
    #     if(sorted_MSOs[j + 1] - sorted_MSOs[j] == 1):
    #         consecutive_MSOs.append(sorted_MSOs[j])
    #         consecutive_MSOs.append(sorted_MSOs[j + 1])
    #     elif()
    # if(len(consecutive_MSOs) > 1):
    #     for j in range(len(consecutive_MSOs)):
    #         value = consecutive_MSOs[j]
    #         while(consecutive_MSOs.count(value) > sorted_MSOs.count(value)):
    #             consecutive_MSOs.remove(value)
    #     print(consecutive_MSOs)

    # have this loop check in the case of collisions
    for j in range(len(collision_objects)):  # for every collision
        MSO = collision_objects[j].MSO
        # print(MSO)
        # SORT = seconds[i].copy()
        # SORT.sort()
        # print(SORT)
        for k in range(p.num_vehicles):  # for every vehicle...
            MSO_to_check = vehicles[k].individual_MSOs[i]
            # print('Im deep')

            # text = "mso to compare {} and mso in question {}"
            # print(text.format(MSO, MSO_to_check))
            # print(MSO_to_check)
            if ((MSO_to_check < (MSO - 8)) or (
                    MSO_to_check > (MSO + 8))):  # that is not part of the collision (and not switching modes)
                # print('Im deeper')
                transmissions = []
                for l in range(len(collision_objects[
                                       j].vehicles_involved)):  # compare to each vehicle that was part of the collision
                    transmissions.append(
                        vehicles[collision_objects[j].vehicles_involved[l]].transmissions[collision_objects[j].second][
                            k])
                transmissions.sort(key=lambda transmission: transmission.distance)
                if transmissions[0].received_power >= p.receiver_min_trigger_level:
                    if transmissions[0].received_power - transmissions[1].received_power >= p.power_difference_dB:
                        for l in range(len(transmissions) - 1):
                            if transmissions[l + 1].successful_decode == True:
                                transmissions[l + 1].successful_decode = False
                                false_from_collision += 1
                                # print("one succeeded")
                    else:
                        for l in range(len(transmissions)):  # make them all False
                            if transmissions[l].successful_decode == True:
                                transmissions[l].successful_decode = False
                                false_from_collision += 1
                else:
                    for l in range(len(transmissions)):  # make them all False
                        if transmissions[l].successful_decode == True:
                            transmissions[l].successful_decode = False
                            false_from_collision += 1
                # now correspond sorted to not. figure out what the difference needs to be and make it a parameter.
                # if(powers_sorted[0] - powers_sorted[1] >= p.power_difference_dBm):
                #    unsuccessful_decodes += len(powers_sorted) - 1
                # elif(powers_sorted[1] - powers_sorted[2] >= 3.0):
                #    unsuccessful_decodes += len(powers_sorted)

for i in range(p.num_vehicles):
    for j in range(p.num_seconds):
        for k in range(p.num_vehicles):
            if vehicles[i].transmissions[j][k] == 0:
                continue
            elif vehicles[i].transmissions[j][k].successful_decode:
                successful_decodes += 1
            elif vehicles[i].transmissions[j][k].successful_decode == False:
                unsuccessful_decodes += 1

# This prints the number of __-way MSO collisions
vehicles_per_collision_bins = 0
counts = []
for i in range(2, p.num_vehicles + 1):  # currently only checks up to 9-way collisions
    count = 0
    count = vehicles_per_collision.count(i)
    if (count != 0):
        counts.append(count)
        vehicles_per_collision_bins += 1
        txt = "Number of {}-way MSO collisions: {}"
        print(txt.format(i, count))

x = np.arange(2, vehicles_per_collision_bins + 2, step=1)

print("Total MSO collisions: ", len(MSO_collisions))

print("Total transmissions: ", unsuccessful_decodes + successful_decodes)
txt = "Unsuccessful decodes / Desired decodes: {}/{} = {} %"
print("Unsuccessful decodes:", unsuccessful_decodes)
print(txt.format(unsuccessful_decodes, p.num_desired_decodes,
                 round(100 * unsuccessful_decodes / p.num_desired_decodes, 2)))
print("Failed from MTL: ", false_from_MTL)
print("Failed from closeness: ", false_from_closeness)
print("Failed from message overlap: ", false_from_message_overlap)
print("Failed from MSO collision: ", false_from_collision)
print("Failed from too high power: ", false_from_high_power)
print(false_from_closeness + false_from_collision + false_from_message_overlap + false_from_MTL + false_from_high_power)
plt.figure(1)
plt.hist(MSOs, bins=3200)
mplcursors.cursor()
plt.title('Occurrences of Specific MSOs')
plt.xlabel('MSO')
plt.ylabel('Occurrences of MSO')

plt.figure(2)
plt.hist(MSO_collisions, bins=3200)
mplcursors.cursor()
plt.title('Occurrences of MSO Collisions on a Specific MSO')
plt.xlabel('MSO')
plt.ylabel('Occurrences of Collisions on MSO')

plt.figure(3)
# plt.hist(vehicles_per_collision, bins=vehicles_per_collision_bins)
plt.bar(x, counts)
plt.xticks(x)
mplcursors.cursor()
plt.title('MSO Collisions by Number of Vehicles Involved')
plt.xlabel('Vehicles per Collision')
plt.ylabel('Number of Collisions')

for i in range(p.num_vehicles):
    continue

plt.rcParams.update({'font.size': 15})
fig = plt.figure(4)
plt.gcf().subplots_adjust(left=0.15)
ax = fig.add_subplot(111)
# set_trace()
stop = p.vehicleStates[i]['p'][:, 0].shape[0]
# plt.waitforbuttonpress()
for i in range(p.num_vehicles):
    # ax.scatter(quadStates[i]['p'][:,0][0:stop:100],quadStates[i]['p'][:,1][0:stop:100],quadStates[i]['p'][:,2][0:stop:100], s=5)
    ax.scatter(p.vehicleStates[i]['p'][:, 0][0:stop:100], p.vehicleStates[i]['p'][:, 1][0:stop:100], s=5)

# ax.view_init(90, 90)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
# ax.set_ylabel('Y (m)')
plt.show()

# plt.figure(4)
# mplcursors.cursor()
# plt.title('New Title')
# plt.xlabel('New X')
# plt.ylabel('New Y')

plt.pause(0.0001)
plt.waitforbuttonpress()

'''
# Puts a list of MSOs in for every second in seconds array
for i in range(p.num_seconds):
    MSOs_per_second = []
    seconds.append(MSOs_per_second) # We have a list for every second
'''

'''
# Populate vehicles list with vehicles
for i in range(p.num_vehicles):
    uas = vehicle(positions[i][0])
    vehicles.append(uas)
'''

# Just checks that powers reach the threshold
'''
    vehicles_sorted = vehicles.copy()
    vehicles_sorted.sort(key=lambda vehicle: vehicle.individual_MSOs[i]) # sort by MSO for each second
    for j in range(p.num_vehicles):
        vehicle_to_transmit = vehicles_sorted[j] # pick a vehicle that is transmitting
        for k in range(j + 1, j + p.num_vehicles): # iterate through all of the other vehicles
            P_r_dBm = pf.combinedReceivedPower(vehicle_to_transmit, vehicles_sorted[k % p.num_vehicles])
            if P_r_dBm >= -93.0: # -93 dBm is the minimum trigger level for the Ping2020i (DO-282B defines similar specs)
                continue #decode is successful? do nothing?
            else:
                unsuccessful_decodes += 1
'''

'''
# This sorts the data and records the number of times
# that a specific MSO occurred in a second. The number
# of collisions recorded is 1, no matter how many
# vehicles were involved
for i in range(p.num_seconds): # for each second
    for j in range(752, 3952): # for each MSO
        count = seconds[i].count(j) # count how many times that MSO occurred in the second
        if count > 1: # if the MSO occurred more than once, there was an MSO collision
            # do something to figure out which vehicles
            vehicles_involved =[] # records which vehicles are involved in the collision
            for k in range(p.num_vehicles): # for each vehicle
                if seconds[i][k] == j: # if the MSO matches the one of the collision
                    vehicles_involved.append(k) # record the vehicle number, which corresponds to an index in the vehicles list
            vehicles_per_collision.append(count) # Record how many vehicles per collision
            MSO_collisions.append(j) # record MSO where collision happened
            new_collision = MSO_collision(i, j, vehicles_involved) # record the collision as object
            collision_objects.append(new_collision) # add the new collision to the list
'''
# What follows is a fun section I developed that is doing everything iteratively...
# you could have this section specifically check in the case of non collisions
'''
vehicles_sorted = vehicles.copy()
for i in range(p.num_seconds):
    vehicles_sorted.sort(key=lambda vehicle: vehicle.individual_MSOs[i]) # sort by MSO for each second
    for j in range(p.num_vehicles):
        vehicle_to_transmit = vehicles_sorted[j] # pick a vehicle that is transmitting
        for k in range(j + 1, j + p.num_vehicles): # iterate through all of the other vehicles
            #distance = np.sqrt((vehicle_to_transmit.LAT[i] - vehicles_sorted[k % p.num_vehicles].LAT[i])**2 + \
            #    (vehicle_to_transmit.LON[i] - vehicles_sorted[k % p.num_vehicles].LON[i])**2) # calculate the distance between the two vehicles
            #distance = pf.distanceBetweenVehicles(vehicle_to_transmit, vehicles_sorted[k % p.num_vehicles], i)
            #P_r_dBm = pf.todBm(pf.powerReceived(pf.powerDensityPt(p.ERP, distance), p.wavelength)) # calculate the power received at the receiver location
            P_r_dBm = pf.combinedReceivedPower(vehicle_to_transmit, vehicles_sorted[k % p.num_vehicles], i)
            if P_r_dBm >= -93: # -93 dBm is the minimum trigger level for the Ping2020i (DO-282B defines similar specs)
                continue #decode is successful? do nothing?
            else:
                unsuccessful_decodes += 1

# have this loop check in the case of collisions
for i in range(len(collision_objects)): # for every collision
    powers = []
    second = collision_objects[i].second
    MSO = collision_objects[i].MSO
    for j in range(p.num_vehicles): # for every vehicle...
        if(vehicles[j].individual_MSOs[second] != MSO): # that is not part of the collision
            for k in range(len(collision_objects[i].vehicles_involved)): # compare to each vehicle that was part of the collision
                P_r_dBm = pf.combinedReceivedPower(vehicles[collision_objects[i].vehicles_involved[k]], vehicles[j], second)
                powers.append(P_r_dBm)
                # now we would do some stuff where we decide which powers are valid, which wins
'''
# so a transmitting vehicle misses the 8 MSOs before and after it's transmission, unless it is one of the first 8 or last 8 MSOs
# basic and long messages alternate 50/50--maybe we can say that odd frame numbers are basic and even are long? Then we randomize frame numbers
# is my handling of the message lengths too simplistic? is it even working?
# FIXME you need to consider now what to do about the ERP, which is resulting in stunningly low received powers
