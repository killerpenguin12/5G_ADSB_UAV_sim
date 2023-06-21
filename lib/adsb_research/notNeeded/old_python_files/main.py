'''
Old version. No longer used. 5/5/2020
'''
import numpy as np
import propagation_param as p
from Vehicle import vehicle
import propagationFunctions
import random
import matplotlib.pyplot as plt
import mplcursors

vehicles = [] # Declare list of vehicles, empty for now
#total_each_MSO = [0] * 3200 # Declare list of MSOs index is the MSO slot
MSOs = [] # Will store every MSO calculated, so we can look at the frequency of each MSO
MSO_collisions = [] # Will record any MSO that is calculated by two or more vehicles in the same second
                # and it will record that MSO once for every other vehicle that calculates it in that second (N-1 MSO collisions for N vehicles)
                # I realize we can define MSO collisions in a more sophisticated way, and maybe even make an MSO
                # collision object that records how many vehicles had the same MSO, what second it was,
                # and what the individual locations of the vehicles were. This is a starting place.
seconds = [] # List that will have a list of MSOs for every second
vehicles_per_collision = [] # Will keep track of how many __-way MSO collisions there are

# Puts a list of MSOs in for every second in seconds array
for i in range(p.num_seconds):
    MSOs_per_second = []
    seconds.append(MSOs_per_second) # We have a list for every second

# Populate vehicles list with vehicles
for i in range(p.num_vehicles):
    # The line below initializes each vehicle with a randoM value between 0 and 4095 inclusive
    # to represent the 12 L.S.B.'s of the latitude and longitude. 2^12 = 4096, so 12 bits can
    # represent values from 0 to 4095.
    uas = vehicle(random.randint(0, 4096), random.randint(0, 4096))
    vehicles.append(uas)

# This calculates the MSO for the first second.
# This model assumes that during the first second,
# all of the UATs are in their first frame, as opposed
# to real life where they would all probably be at random
# frame numbers.
for i in range(p.num_vehicles):
    MSO = vehicles[i].fullMSORange() # calculate MSO
    MSOs.append(MSO) # record MSO
    seconds[0].append(MSO)
    #total_each_MSO[MSO - 752] += 1

# The rest of the time we do the other MSO calculation
for i in range(1, p.num_seconds):
    # Before calculating MSOs, update the vehicle locations
    # Because I don't have GPS data to work with, for now
    # I am just assigning a random location to each vehicle
    # every second.
    for j in range(p.num_vehicles):
        vehicles[j].updateLatLon(random.randint(0, 4096), random.randint(0, 4096))

    # Begin MSO calculations
    for j in range(p.num_vehicles):
        MSO = vehicles[j].restrictedMSORange() # calculate MSO
        MSOs.append(MSO) # record MSO
        seconds[i].append(MSO)
        #total_each_MSO[MSO - 752] += 1

# This sorts the data and records the number of times
# that a specific MSO occurred in a second. The number
# of collisions recorded is the number of UATs
# in the message collision - 1, which can be changed.
for i in range(p.num_seconds):
    for j in range(752, 3952):
        count = seconds[i].count(j)
        if count > 1:
            vehicles_per_collision.append(count)
            for k in range(count - 1):
                MSO_collisions.append(j)

# This prints the number of __-way collisions
for i in range(2, 10):
    count = 0
    count = vehicles_per_collision.count(i)
    txt = "Number of {}-way collisions: {}"
    print(txt.format(i, count))

plt.figure(1)
plt.hist(MSOs, bins=3200)
mplcursors.cursor()
plt.title('Frequencies of Specific MSOs During Simulation')
plt.xlabel('MSO')
plt.ylabel('Frequency of MSO')

plt.figure(2)
plt.hist(MSO_collisions, bins=3200)
mplcursors.cursor()
plt.title('Frequencies of MSO Collisions on a Specific MSO')
plt.xlabel('MSO')
plt.ylabel('Collisions Occurring on MSO')

plt.figure(3)
plt.hist(vehicles_per_collision)
plt.title('Number of MSO Collisions with a Certain Number of Vehicles')
plt.xlabel('Vehicles per Collision')
plt.ylabel('Number of Collisions in Simulation')

plt.pause(0.0001)
plt.waitforbuttonpress()

# In the future, you will want a loop that executes
# all of the actions for one second for a single drone
# every iteration

# hex(integer)
# int(hex, 16)
# I assume for now a range of 0 to 4095 for an unsigned hex value

# Questions
# k question
# How do frames work? Are they based of UTC time? i.e. If UTC is 10:43:32 then the frame is 32? Do frames ever start over?
# The MSO algorithms make me think that frames start with 0 at power up, and then increase until power down. Is that the case?
# How is the arithmetic performed on the bits? are the 12 bits converted to a decimal number first?
# what format do I receive GPS coordinates in hex?


### OLD SORTING STUFF ###
# Ignore the next bit, I didn't know how to sort the data yet
'''
# Now we sift through the data. There may be better ways to sort,
# in fact, I have already had some better ideas
for i in range(p.num_seconds): # for every second
    for j in range(p.num_vehicles): # for every vehicle in every second
        MSO_to_check = (vehicles[j].individual_MSOs)[i] # choose the MSO of the jth vehicle in that second
        skip_MSO = False # reset flag
        for k in range(len(MSO_collisions)): # now we iterate through the recorded collisions to make sure we aren't
            if MSO_collisions[k] == MSO_to_check:
                skip_MSO = True
                break
        for k in range(j + 1, p.num_vehicles):
            if skip_MSO == True:
                break
            if MSO_to_check == (vehicles[k].individual_MSOs)[i]:
                MSO_collisions.append(MSO_to_check)

for i in range(p.num_seconds):
    for j in range(p.num_vehicles*i, p.num_vehicles*(i + 1)):
        MSO_to_count = MSOs[j]
        for k in range(j + 1, p.num_vehicles + j):
            if MSO_to_count == MSOs[k]
        MSOs.count()
'''
