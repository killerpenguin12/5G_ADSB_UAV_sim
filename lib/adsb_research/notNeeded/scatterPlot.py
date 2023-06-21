### imports all of the correct files and stuff for 3D graphing
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import time
from random import seed
from random import randint
from datetime import datetime
import matplotlib.image as mpimg
import sys
sys.path.append('/Users/themagician/Desktop/researchCammy/adsb_research')
from vehicle import vehicle

### Starting positions based off real world lat/lon/alt
startLat = 0
startLon = 0
startAlt = 0

spacing = 1    ### Minimum spacing (in meters) between vehicles
numVehicles = 100    ### Number of vehicles in current simulation
maxMPS = 250    ### Maximum velocity of each vehicle in meters/second
random = True    ### Decides if vehicle start/end coordinates are random

### Range of simulation distances (in meters)
maxRange = 74080
maxLat = maxRange
maxLon = maxRange
maxAlt = maxRange

vehicleList = []    ### Holds all vehicle objects in one list
startCoord = []    ### Holds the starting coordinate position for each respective vehicle
endCoord = []    ### Holds the ending coordinate position for each respective vehicle
msoList = [None]*(3200+752)    ### Holds all possible MSO time slots
msoOverlapList = []   ### Holds the MSO time slots where collisions occur
collisionCount = 0    ### Counts how many collisions happen in the simulation


# ### Creates a coordinate list for every point in the plane
# x = startLat
# y = startLon
# z = startAlt
# for x in range(startLat, startLat+maxLat):
#     for y in range(startLon, startLon+maxLon):
#         if ((x%spacing == 0) and (y%spacing == 0)):
#             newTup = (x,y,0)
#             coordList.append(newTup)

### Creates a random start and end coordinate position for each vehicle
if random == True:
    seed(datetime.now())
    for i in range(0,numVehicles):
        ### The starting coordinate position is randomly chosen inside the simulation limits
        startTup = (randint(maxMPS,maxRange-maxMPS),randint(maxMPS,maxRange-maxMPS),0)
        startCoord.append(startTup)
        ### A new vehicle is now instantiated with that random starting coordinate and appended to the vehicle list
        uas = vehicle(startCoord[i])
        vehicleList.append(uas)
        ### The ending coordinate position is randomly chosen inside the simulation limits
        endTup = (randint(startCoord[i][0]-maxMPS,startCoord[i][0]+maxMPS),randint(startCoord[i][1]-maxMPS,startCoord[i][1]+maxMPS),0)
        endCoord.append(endTup)
        ### Each vehicle is updated with the end coordinate position and another random number is calculated
        vehicleList[i].updatePosition(endTup)
        vehicleList[i].getNewMval(randint(0,maxRange))
else:
    ### This vehicle is selected as the first position
    startCoord[0] = coordList[10]
    for i in range(0,len(coordList)):
        initVehicle = vehicle(startCoord[i])
        vehicleList.append(initVehicle)
    for i in range(0,len(coordList)):
        vehicleList[i].updatePosition(coordList[i])
        vehicleList[i].pseudoRandomNumberR(0)

### Gets the MSO value of each vehicle
for i in range(len(vehicleList)):
    msoValue = vehicleList[i].fullMSORange()    ### Returns the current vehicle's MSO value
    ### If the msoList already has the slot filled by another vehicle, then there is a collision
    if (msoList[msoValue] != None):
        msoList[msoValue].append(vehicleList[i].getPosition())
        collisionCount += 1
    ### No collision for current MSO slot and it is added by the current vehicle
    else:
        msoList.pop(msoValue)
        msoList.insert(msoValue,[vehicleList[i].getPosition()])

### Checks for overlapping messages because back to back MSO time slots will overlap
for i in range(len(msoList)):
    if (i == 0):
        print("Nothing")
    else:
        if (msoList[i-1] != None) and (msoList[i] != None):
            print("Here")
            msoOverlapList.append([i-1,i])
            collisionCount += 1

print(msoOverlapList)
### Analysis of MSO collisions and their specific values
for i in range(len(msoList)):
    if (msoList[i] != None) and (len(msoList[i]) > 1):
        print(len(msoList[i]),"Vehicles calculated the MSO time slot",i,":",msoList[i])
print("Number of Total Collisions:",collisionCount)

### Plot constraints
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim([-1,maxRange+1])
ax.set_ylim([-1,maxRange+1])

### Connects the start and end coordinates in order to plot as a pair
xconCoord = []
yconCoord = []
for i in range(numVehicles):
    xconCoord.append([startCoord[i][0],endCoord[i][0]])    ### Start and end x-coordinates are paired here
    yconCoord.append([startCoord[i][1],endCoord[i][1]])    ### Start and end y-coordinates are paired here

### Plots the pairs together and connects them with a dashed line to show connection
for i in range(numVehicles):
    start = ax.scatter(xconCoord[i][0], yconCoord[i][0], c='b', marker='x',label="Start")    ### Start coordinates are plotted
    end = ax.scatter(xconCoord[i][1], yconCoord[i][1], c='b', marker='.',label="End")    ### End coordinates are plotted
    ax.plot(xconCoord[i], yconCoord[i], color='gray', linestyle='dashed', linewidth=1, markersize=1)    ### Start and end coordinates are connected

### Plots all collisions in the simulation and connects them together
for i in range(len(msoList)):
    if (msoList[i] != None) and (len(msoList[i]) > 1):
        xcollisionCoord = []
        ycollisionCoord = []
        ### The coordinates of the MSO collisions at its specific time slot are plotted
        for j in range(len(msoList[i])):
            myCollisions = ax.scatter(msoList[i][j][0], msoList[i][j][1], c='r', marker='o', label="Collided")
            xcollisionCoord.append(msoList[i][j][0])    ### All x-coordinates at current MSO slot are paired
            ycollisionCoord.append(msoList[i][j][1])    ### All y-coordinates at current MSO slot are paired
        ### Here, the paired coordinates at the current MSO time slot are connected together
        ax.plot(xcollisionCoord, ycollisionCoord, color='black', linestyle='dashed', linewidth=1, markersize=1)

# img = mpimg.imread("/Users/themagician/Desktop/provo.png")
# plt.imshow(img,origin='upper',aspect='auto',extent=[0, 400, 0, 400])
plt.xlabel("Longitude (Meters)")
plt.ylabel("Latitude (Meters)")
### Gives the legend, even if no collisions happen (a point outside plot range is given to avoid confusion on plot)
if collisionCount == 0:
    myCollisions = ax.scatter(maxRange+100, maxRange+100, c='r', marker='o', label="Collided")
    plt.legend(handles=[start,end,myCollisions],loc='upper left', frameon=True)
else:
    plt.legend(handles=[start, end, myCollisions],loc='best', frameon=True)
plt.show()
