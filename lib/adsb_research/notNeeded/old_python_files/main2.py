'''
Old version, no longer used 5/5/2020
'''
import numpy as np
import propagation_param as p
from Vehicle import vehicle
import propagationFunctions
import random
import matplotlib.pyplot as plt
import mplcursors

vehicles = [] # Declare list of vehicles, empty for now
MSOs = [] # Will store every MSO calculated, so we can look at the frequency of each MSO
seconds = [] # List that will have a list of MSOs for every second, to organize them better
MSO_collisions = [] # Will record any MSO that is calculated by two or more vehicles in the same second
vehicles_per_collision = [] # Will keep track of how many __-way MSO collisions there are

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
    if(i > 0): # dont update for first second
        for j in range(p.num_vehicles):
            vehicles[j].updateLatLon(random.randint(0, 4096), random.randint(0, 4096))

    # Begin MSO calculations
    for j in range(p.num_vehicles):
        MSO = vehicles[j].fullMSORange() # calculate MSO
        vehicles[j].m += 1 # increase frame
        MSOs.append(MSO) # record MSO
        seconds[i].append(MSO) # record MSO in a way that helps with collisions

# This sorts the data and records the number of times
# that a specific MSO occurred in a second. The number
# of collisions recorded is 1, no matter how many
# vehicles were involved
for i in range(p.num_seconds): # for each second
    for j in range(752, 3952): # for each MSO
        count = seconds[i].count(j) # count how many times that MSO occurred in the second
        if count > 1: # if the MSO occurred more than once, there was an MSO collision
            vehicles_per_collision.append(count) # Record how many vehicles per collision
            MSO_collisions.append(j) # record MSO where collision happened

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

# In the future, you will want a loop that executes
# all of the actions for one second for a single drone
# every iteration

# hex(integer)
# int(hex, 16)
# I assume for now a range of 0 to 4095 for an unsigned hex value

# Questions
# k question
# FIXME is R(k) constant? Or is k just constant and R(k) is calculated every time?
        # My interpretation is that k is constant because R(m-1) is defined as a number,
        # but for R(k), k is just defined as a frame--not R(k) being defined as a number
# How do frames work? Are they based of UTC time? i.e. If UTC is 10:43:32 then the frame is 32? Do frames ever start over?
# The MSO algorithms make me think that frames start with 0 at power up, and then increase until power down. Is that the case?
# How is the arithmetic performed on the bits? are the 12 bits converted to a decimal number first?
# what format do I receive GPS coordinates in hex?

# Checks if any MSO was not used
'''
for i in range(752, 3952):
    count = 1
    count = MSOs.count(i)
    if(i == 0):
        txt = "MSO {} was never selected."
        print(txt.format(i))
'''

# Restricted range MSO calculation
'''
# This calculates the MSO for restricted range
for i in range(p.num_vehicles):
    MSO = vehicles[i].restrictedMSOrange() # calculate MSO
    MSOs.append(MSO) # record MSO
    seconds[0].append(MSO)
    #total_each_MSO[MSO - 752] += 1
'''

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
