'''
Another main file no longer in use. 5/5/2020
'''
import numpy as np
import propagation_param as p
from Vehicle import vehicle
import propagationFunctions as pf
import random
import matplotlib.pyplot as plt
import mplcursors
from MSO_collision import MSO_collision, vehicle_power

vehicles = [] # Declare list of vehicles, empty for now
#vehicles_sorted = [] # the vehicles, sorted by MSO for the given second
MSOs = [] # Will store every MSO calculated, so we can look at the frequency of each MSO
seconds = [] # List that will have a list of MSOs for every second, to organize them better
MSO_collisions = [] # Will record any MSO that is calculated by two or more vehicles in the same second
vehicles_per_collision = [] # Will keep track of how many __-way MSO collisions there are
unsuccessful_decodes = 0

def vehiclesKey(vehicle_in, index):
    return vehicle_in.individual_MSOs[index]

# Puts a list of MSOs in for every second in seconds array
for i in range(p.num_seconds):
    MSOs_per_second = []
    seconds.append(MSOs_per_second) # We have a list for every second

# Populate vehicles list with vehicles
for i in range(p.num_vehicles):
    # The line below initializes each vehicle with a random value between 0 and 4095 inclusive
    # to represent the 12 L.S.B.'s of the latitude and longitude. 2^12 = 4096, so 12 bits can
    # represent values from 0 to 4095.
    uas = vehicle(random.randint(0, 4096), random.randint(0, 4096))
    vehicles.append(uas)

# The rest of the time we do the other MSO calculation
for i in range(p.num_seconds):
    # Before calculating MSOs, update the vehicle locations.
    # Because I don't have GPS data to work with, for now
    # I am just assigning a random location to each vehicle
    # every second.
    collision_objects = []
    if(i > 0): # dont update for first second
        for j in range(p.num_vehicles):
            vehicles[j].updateLatLon(random.randint(0, 4096), random.randint(0, 4096))

    # Begin MSO calculations
    for j in range(p.num_vehicles):
        MSO = vehicles[j].fullMSORange() # calculate MSO
        vehicles[j].m += 1 # increase frame
        MSOs.append(MSO) # record MSO
        seconds[i].append(MSO) # record MSO in a way that helps with collisions

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

    # have this loop check in the case of collisions
    for j in range(len(collision_objects)): # for every collision
        MSO = collision_objects[j].MSO
        for k in range(p.num_vehicles): # for every vehicle...
            if(vehicles[k].individual_MSOs[i] != MSO): # that is not part of the collision (these will be receiving, as they are not transmitting)
                powers = []
                for l in range(len(collision_objects[j].vehicles_involved)): # compare to each vehicle that was part of the collision
                    P_r_dBm = pf.combinedReceivedPower(vehicles[collision_objects[j].vehicles_involved[l]], vehicles[k])
                    new_power = vehicle_power(P_r_dBm, collision_objects[j].vehicles_involved[l], pf.distanceBetweenVehicles(vehicles[collision_objects[j].vehicles_involved[l]], vehicles[k]))
                    powers.append(new_power)
                    #powers.append(P_r_dBm)
                    # now we would do some stuff where we decide which powers are valid, which wins
                powers_sorted = powers.copy()
                powers_sorted.sort(key=lambda vehicle_power: vehicle_power.distance)
                unsuccessful_decodes += pf.determineUnsuccessfulDecodes(powers_sorted, 0)

                # now correspond sorted to not. figure out what the difference needs to be and make it a parameter.

                #if(powers_sorted[0] - powers_sorted[1] >= p.power_difference_dBm):
                #    unsuccessful_decodes += len(powers_sorted) - 1
                #elif(powers_sorted[1] - powers_sorted[2] >= 3.0):
                #    unsuccessful_decodes += len(powers_sorted)




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
# This prints the number of __-way MSO collisions
vehicles_per_collision_bins = 0
counts = []
for i in range(2, p.num_vehicles + 1): # currently only checks up to 9-way collisions
    count = 0
    count = vehicles_per_collision.count(i)
    if(count != 0):
        counts.append(count)
        vehicles_per_collision_bins += 1
        txt = "Number of {}-way MSO collisions: {}"
        print(txt.format(i, count))

x = np.arange(2, vehicles_per_collision_bins + 2, step=1)

print("Total MSO collisions: ", len(MSO_collisions))

# What follows is a fun section I developed that is doing everything iteratively...
'''
# you could have this section specifically check in the case of non collisions
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

txt = "Unsuccessful decodes / Desired decodes: {}/{} = {}"
print("Unsuccessful decodes:", unsuccessful_decodes)
print(txt.format(unsuccessful_decodes, p.num_desired_decodes, 100*unsuccessful_decodes/p.num_desired_decodes))

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
#plt.hist(vehicles_per_collision, bins=vehicles_per_collision_bins)
plt.bar(x, counts)
plt.xticks(x)
mplcursors.cursor()
plt.title('MSO Collisions by Number of Vehicles Involved')
plt.xlabel('Vehicles per Collision')
plt.ylabel('Number of Collisions')

plt.pause(0.0001)
plt.waitforbuttonpress()

# What do we do if there is a collision? We should probably just avoid physical collisions?
