### imports all of the correct files and stuff for 3D graphing
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sb
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
simLength = 1#600    ### Number of frames in simulation
numSims = 3   ### Number of simulations
simCount = 0
maxMPS = 10    ### Maximum velocity of each vehicle in meters/second
frameVal = 0
random = False
sameFrame = True
showAllPoints = True
noNorthSouth = False
noEastWest = False
flyNorth = True
flyEast = True

### Range of simulation distances (in meters)
maxRange = 1000
maxLat = maxRange
maxLon = maxRange
maxAlt = maxRange
lat0 = 40.2338
lon0 = -111.658531
binaryToDegree = 2.1457672*10**(-5)
degreeToMetersLat = 110567 + int(113*(lat0/9))
degreeToMetersLon = int((degreeToMetersLat)*np.cos(lat0*180/np.pi))
#Adjustment for 753 origin offset
binaryToMetersLat = binaryToDegree*degreeToMetersLat*.775
binaryToMetersLon = binaryToDegree*degreeToMetersLon*1.31
1.8470935718976 #Lat
2.38377419578696 #Lon
print(binaryToMetersLat)
print(binaryToMetersLon)
print("enter to continue")
input("")
numCalculations = int(((maxRange//binaryToMetersLon)+1)*((maxRange//binaryToMetersLat)+1))

vehicleList = []    ### Holds all vehicle objects in one list
startCoord = []    ### Holds the starting coordinate position for each respective vehicle
endCoord = []    ### Holds the ending coordinate position for each respective vehicle
msoOverlapList = []   ### Holds the MSO time slots where collisions occur
averageCollisions = []
collisionCount = 0

x = []
y = []

### CHECK to see if the collisions based off of the current frame value

################################################################################################################################################################
def initCalculatedVehicles():
    seed(datetime.now())
    x = 0
    y = 0
    for i in range(numCalculations):
        # print("Initializing...")
        change = False
        if (x*binaryToMetersLon > maxRange):
            x = 0
            y += 1
            change = True
        ### The starting coordinate position is randomly chosen inside the simulation limits
        if random:
            startTup = (randint(maxMPS,maxRange-maxMPS),randint(maxMPS,maxRange-maxMPS),0)
        else:
            startTup = (x*binaryToMetersLon,y*binaryToMetersLat,0)
        startCoord.append(startTup)
        ### A new vehicle is now instantiated with that random starting coordinate and appended to the vehicle list
        mVal = randint(0,1800)
        if sameFrame:
            mVal = frameVal
        uas = vehicle(startCoord[i],frame=mVal)    ### No vehicle is in the air for more than an hour
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

        if random:
            endTup = (startCoord[i][0]+randint(-maxMPS,maxMPS),startCoord[i][1]+randint(-maxMPS,maxMPS),0)
        else:
            endTup = (startCoord[i][0]+maxMPS*signChangeX,startCoord[i][1]+maxMPS*signChangeY,0)

        endCoord.append(endTup)
        ### Each vehicle is updated with the end coordinate position and another random number is calculated
        vehicleList[i].updatePosition(endTup)
        vehicleList[i].incrementMval()
        if i != 0:
            if (vehicleList[i].getN0() == vehicleList[i-1].getN0()):
                print("False")
            print("N_1:",vehicleList[i].getN0())
        x += 1

################################################################################################################################################################
def updateCalculatedVehiclePositions():
    global simCount
    for i in range(numCalculations):
        seed(datetime.now())
        ### Each vehicle is updated with the end coordinate position and another random number is calculated
        vehicleList[i].updatePosition(endCoord[i])
        mVal = randint(0,1800)
        if sameFrame:
            mVal = frameVal
        vehicleList[i].pseudoRandomNumberR(mVal)
        vehicleList[i].incrementMval()

################################################################################################################################################################
def printVehicleValues(initVehicle,collideVehicle):
    print("\nInitial Vehicle:")
    print("Frame:",initVehicle.getMval())
    print("N_0:",initVehicle.getN0())
    print("N_1:",initVehicle.getN1())
    print("R(m-1):",initVehicle.getR_d1())
    prc = ((4001*initVehicle.getR_d1() + initVehicle.leastSignificantBits(initVehicle.getMval() % 2)) % 3200)
    print("Pseudo Random Calculator:",prc)
    print("R(m):",initVehicle.getR())

    print("\nCollided Vehicle:")
    print("Frame:",collideVehicle.getMval())
    print("N_0:",collideVehicle.getN0())
    print("N_1:",collideVehicle.getN1())
    print("R(m-1):",collideVehicle.getR_d1())
    prc2 = ((4001*collideVehicle.getR_d1() + collideVehicle.leastSignificantBits(collideVehicle.getMval() % 2)) % 3200)
    print("Pseudo Random Calculator:",prc2)
    print("R(m):",collideVehicle.getR())
    print("\n[R(m-1)Init] - [R(m-1)Collide]:",initVehicle.getR_d1() - collideVehicle.getR_d1())
    print("[N(1)Init] - [N(1)Collide]:",initVehicle.getN1() - collideVehicle.getN1())
    print("[N(0)Init] - [N(0)Collide]:",initVehicle.getN0() - collideVehicle.getN0())


################################################################################################################################################################
def getVehicleMSOs():
    ### Gets the MSO value of each vehicle
    global collisionCount
    initialMSO = vehicleList[0].fullMSORange() - 752
    for i in range(numCalculations):
        print("Working...")
        if i == 0:
            continue
        msoValue = vehicleList[i].fullMSORange() - 752   ### Returns the current vehicle's MSO value
        ### If the msoList already has the slot filled by another vehicle, then there is a collision
        if (msoValue == initialMSO):
            printVehicleValues(vehicleList[0],vehicleList[i])
            # print("Press enter to continue...")
            # input("")
            vehicleList[0].incrementMval()
            vehicleList[i].incrementMval()
            printVehicleValues(vehicleList[0],vehicleList[i])
            # print("Press enter to continue...")
            # input("")
            x.append(endCoord[i][0])    ### Start and end x-coordinates are paired here
            y.append(endCoord[i][1])    ### Start and end y-coordinates are paired here
            collisionCount += 1

################################################################################################################################################################
def plot():
    ### Plot constraints
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim([-1,maxRange+1])
    ax.set_ylim([-1,maxRange+1])
    print("Plotting...")
    plt.hist2d(x,y, bins=[np.arange(-maxMPS-1,maxRange+maxMPS+1,binaryToMetersLon),np.arange(-maxMPS-1,maxRange+maxMPS+1,binaryToMetersLat)],cmap='YlOrRd')
    plt.show(block=False)
    plt.pause(1)
    plt.close

################################################################################################################################################################
### Simulation loop
initCalculatedVehicles()

for j in range(numSims):
    simCount += 1
    for i in range(simLength):
        seed(datetime.now())
        updateCalculatedVehiclePositions()
        getVehicleMSOs()
        # calculateNumCollisions()
        print("Simulation:",j)
        print("Total Collisions:",collisionCount)
    averageCollisions.append(collisionCount/simLength)

totalAverage = 0
sum = 0
for i in range(len(averageCollisions)):
    sum += averageCollisions[i]
totalAverage = sum/len(averageCollisions)
print("\n")
print("Time of Simulation:",simLength/60,"minutes")
print("Number of Simulations:",numSims)
print("Total Collisions:",collisionCount)
print("Average collisions/simulation:",totalAverage)
print("Area:",maxRange,"m^2")
print("\n")
plot()
print("Press enter to exit")
input("")
