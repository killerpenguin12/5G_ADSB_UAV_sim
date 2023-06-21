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
simSize = 1  ### Number of frames in simulation
numVehicles = 100  ### Number of vehicles in current simulation
maxMPS = 100    ### Maximum velocity of each vehicle in meters/second
initVehicles = True    ### For first loop run
flyUp = True
flyRight = True

### Range of simulation distances (in meters)
maxRange = 10000 #740800
maxLat = maxRange
maxLon = maxRange
maxAlt = maxRange

vehicleList = []    ### Holds all vehicle objects in one list
startCoord = []    ### Holds the starting coordinate position for each respective vehicle
endCoord = []    ### Holds the ending coordinate position for each respective vehicle
msoOverlapList = []   ### Holds the MSO time slots where collisions occur
collisionCount = 0

### CHECK to see if the collisions based off of the current frame value

################################################################################################################################################################
def initRandomVehicles():
    seed(datetime.now())
    for i in range(0,numVehicles):
        ### The starting coordinate position is randomly chosen inside the simulation limits
        startTup = (randint(maxMPS,maxRange-maxMPS),randint(maxMPS,maxRange-maxMPS),0)
        startCoord.append(startTup)
        ### A new vehicle is now instantiated with that random starting coordinate and appended to the vehicle list
        uas = vehicle(startCoord[i],frame=randint(0,10000))
        vehicleList.append(uas)
        ### The ending coordinate position is randomly chosen inside the simulation limits
        endTup = (randint(startCoord[i][0]-maxMPS,startCoord[i][0]+maxMPS),randint(startCoord[i][1]-maxMPS,startCoord[i][1]+maxMPS),0)
        endCoord.append(endTup)
        ### Each vehicle is updated with the end coordinate position and another random number is calculated
        vehicleList[i].updatePosition(endTup)
        vehicleList[i].getNewMval(randint(0,maxRange))
        if simSize != 1:
            startCoord[i] = endCoord[i]

################################################################################################################################################################
def updateRandomVehiclePositions():
    for i in range(0,numVehicles):
        ### The ending coordinate position is randomly chosen inside the simulation limits
        endTup = (randint(startCoord[i][0]-maxMPS,startCoord[i][0]+maxMPS),randint(startCoord[i][1]-maxMPS,startCoord[i][1]+maxMPS),0)
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
def calculateNumCollisions():
    ### Analysis of MSO collisions and their specific values
    for i in range(len(msoList)):
        if (msoList[i] != None) and (len(msoList[i]) > 1):
            nVal = []
            for j in range(len(frameList[i])):
                nVal.append(frameList[i][j]%2)
            print(len(msoList[i]),"Vehicles calculated the MSO time slot",i,":",msoList[i],"using N(",nVal,")")
    print("\n")
    if simSize != 1:
        msoList.clear()

################################################################################################################################################################
### Simulation loop
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
    calculateNumCollisions()
print("Total Collisions:",collisionCount)
print("Average number of collisions per second with",numVehicles,"vehicles:",collisionCount/simSize)

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
