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
simSize = 1    ### Number of frames in simulation (10 minutes)
numSims = 1   ### Number of simulations
numVehicles = 500  ### Number of vehicles in current simulation
maxMPS = 100    ### Maximum velocity of each vehicle in meters/second
initVehicles = True    ### For first loop run
noNorthSouth = False
noEastWest = False
flyNorth = True
flyEast = False

### Range of simulation distances (in meters)
maxRange = 40000 #740800
maxLat = maxRange
maxLon = maxRange
maxAlt = maxRange
lonDigit = 1.82609    ### Value (in meters) of one binary digit change in longitude in Provo
latDigit = 2.38264    ### Value (in meters) of one binary digit change in latitude in Provo

vehicleList = []    ### Holds all vehicle objects in one list
startCoord = []    ### Holds the starting coordinate position for each respective vehicle
endCoord = []    ### Holds the ending coordinate position for each respective vehicle
averageCollisions = []
collisionCount = 0

### CHECK to see if the collisions based off of the current frame value

################################################################################################################################################################
def initRandomVehicles():
    seed(datetime.now())
    for i in range(0,numVehicles):
        print("Initializing...")
        ### The starting coordinate position is randomly chosen inside the simulation limits
        startTup = (randint(maxMPS,maxRange-maxMPS),randint(maxMPS,maxRange-maxMPS),0)
        startCoord.append(startTup)
        ### A new vehicle is now instantiated with that random starting coordinate and appended to the vehicle list
        uas = vehicle(startCoord[i],frame=randint(0,1800))    ### No vehicle is in the air for more than an hour
        vehicleList.append(uas)
        ### The ending coordinate position is randomly chosen inside the simulation limits
        if flyNorth:
            signChangeY = 1
        else:
            signChangeY = -1
        if flyEast:
            signChangeX = 1
        else:
            signChangeX = -1

        if noNorthSouth:
            signChangeY = 0
        if noEastWest:
            signChangeX = 0

        endTup = (startCoord[i][0]+maxMPS*signChangeX,startCoord[i][1]+maxMPS*signChangeY,0)
        endCoord.append(endTup)
        ### Each vehicle is updated with the end coordinate position and another random number is calculated
        vehicleList[i].updatePosition(endTup)
        vehicleList[i].incrementMval()
        if simSize != 1:
            startCoord[i] = endCoord[i]

################################################################################################################################################################
def updateRandomVehiclePositions():
    for i in range(0,numVehicles):
        print("Updating...")
        if flyNorth:
            signChangeY = 1
        else:
            signChangeY = -1

        if flyEast:
            signChangeX = 1
        else:
            signChangeX = -1

        if noNorthSouth:
            signChangeY = 0
        if noEastWest:
            signChangeX = 0

        ### The ending coordinate position is randomly chosen inside the simulation limits
        endTup = (startCoord[i][0]+maxMPS*signChangeX,startCoord[i][1]+maxMPS*signChangeY,0)
        ### Each vehicle is updated with the end coordinate position and another random number is calculated
        vehicleList[i].updatePosition(endTup)
        vehicleList[i].incrementMval()
        startCoord[i] = endTup

################################################################################################################################################################
def getVehicleMSOs():
    ### Gets the MSO value of each vehicle
    global collisionCount
    global count
    for i in range(len(vehicleList)):
        print("Working...")
        msoValue = vehicleList[i].fullMSORange() - 752   ### Returns the current vehicle's MSO value
        ### If the msoList already has the slot filled by another vehicle, then there is a collision
        if (msoList[msoValue] != None):
            msoList[msoValue].append(vehicleList[i].getPosition())
            frameList[msoValue].append(vehicleList[i].getMval())
            collisionCount += 1
            count += 1
        ### No collision for current MSO slot and it is added by the current vehicle
        else:
            msoList.pop(msoValue)
            msoList.insert(msoValue,[vehicleList[i].getPosition()])
            frameList.pop(msoValue)
            frameList.insert(msoValue,[vehicleList[i].getMval()])

################################################################################################################################################################
### Simulation loop
for j in range(numSims):
    seed(datetime.now())
    collisionCount = 0
    for i in range(simSize):
        seed(datetime.now())
        msoList = [None]*3200    ### Holds all possible MSO time slots
        frameList = [None]*3200
        count = 0
        if initVehicles:
            initRandomVehicles()
            initVehicles = False
        else:
            updateRandomVehiclePositions()
        getVehicleMSOs()
    print("Simulation:",j)
    print("Total Collisions:",collisionCount)
    print("Average collisions/second with",numVehicles,"vehicles:",collisionCount/simSize)
    averageCollisions.append(collisionCount/simSize)
totalAverage = 0
sum = 0
for i in range(len(averageCollisions)):
    sum += averageCollisions[i]
totalAverage = sum/len(averageCollisions)
print("\n")
print("Time of Simulation:",simSize/60,"minutes")
print("Number of Simulations:",numSims)
print("Average collisions/second:",totalAverage)
print("Area:",maxRange,"m^2")
print("\n")
print("Press enter to exit")
input("")

################################################################################################################################################################
if simSize == 1:
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
        end = ax.scatter(xconCoord[i][1], yconCoord[i][1], c='gray', marker='+',label="End")    ### End coordinates are plotted
        ax.plot(xconCoord[i], yconCoord[i], color='gray', linestyle='dashed', linewidth=1, markersize=1)    ### Start and end coordinates are connected

    ### Plots all collisions in the simulation and connects them together
    for i in range(len(msoList)):
        if (msoList[i] != None) and (len(msoList[i]) > 1):
            xcollisionCoord = []
            ycollisionCoord = []
            ### The coordinates of the MSO collisions at its specific time slot are plotted
            for j in range(len(msoList[i])):
                myCollisions = ax.scatter(msoList[i][j][0], msoList[i][j][1], c='r', marker='.', label="Collided")
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
    plt.show(block=False)
    print("Press enter to exit")
    input("")
