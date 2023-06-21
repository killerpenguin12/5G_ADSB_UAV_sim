### imports all of the correct files and stuff for 3D graphing
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sb
import time
import pymap3d as pymap3d
from dec2bin import dec2bin
from random import seed
from random import randint
from datetime import datetime
import matplotlib.image as mpimg
from dec2bin import dec2bin

LSB = 360./(2.**24.)
binaryDigitCycle = 4096
lat0 = 40.2338
lon0 = -111.658531
h0 = 0 # m, altitude above sea level Provo
initCoord = pymap3d.ned2geodetic(0, 0, 0, lat0, lon0, h0, pymap3d.Ellipsoid('wgs84'))
binaryToDegree = 2.1457672*10**(-5)
degreeToMetersLat = 110567 + int(113*(lat0/9))
degreeToMetersLon = int((degreeToMetersLat)*np.cos(lat0*180/np.pi))
binaryToMetersLat = binaryToDegree*degreeToMetersLat*.775
binaryToMetersLon = binaryToDegree*degreeToMetersLon*1.31

################################################################################################################################################################
def binaryEncodeLSBs(value):
    binValue = value
    if(binValue < 0):
        binValue += 180 # negative values (South for LAT, West for LON)
                        # have slightly different encoding than positive
    binValue = binValue/LSB # LSB is a conversion factor in D0-282B. 360/(2^24)
    binValue = np.round_(binValue) # round to the nearest integer to prep for binary conversion
    binValue = np.int(binValue) # change to an int for binary conversion, otherwise the format is weird
    binValue = dec2bin(binValue) # converts to a binary string
    binValue = binValue[(len(binValue) - 12):len(binValue)] # discards all but the last 12 bits
    binValue = int(binValue, 2) # turns the 12 bit binary string into an integer value
    return binValue # returns the correctly encoded 12 L.S.B.s of the value

################################################################################################################################################################
def pseudoRandomNumberR(long,lat,m,R_m_d1):
    if (m%2 == 1):
        lsbVal = long
    else:
        lsbVal = lat
    if m == 0:
        R_m_d1 = (lat % 3200) # calculate pseudo-random number
        return R_m_d1
    elif m >= 1:
        R_m_d1 = ((4001*R_m_d1 + lsbVal % binaryDigitCycle) % 3200) # calculate pseudo-random number
        # R_m_d1 = ((4001*R_m_d1 + leastSignificantBits(m % 2)) % 3200) # calculate pseudo-random number
        return R_m_d1
    else:
        print('ERROR: m is invalid in R(m)')

################################################################################################################################################################
def fullMSORange(long,lat,m,R_m_d1):
    MSO = (752 + pseudoRandomNumberR(long,lat,m,R_m_d1))   ###DO WE EVEN NEED THIS?????
    return MSO

################################################################################################################################################################
maxRange = 4096
maxMPS = 100
numSims = 1
frameVal = 1
random = False
sameFrame = True
latMovement = True
lonMovement = True
flyNorth = True
flyEast = True
numLatIncrements = int(maxRange//binaryToMetersLat)+1
numLonIncrements = int(maxRange//binaryToMetersLon)+1
numCalculations = int(numLatIncrements*numLonIncrements)
N_0 = binaryEncodeLSBs(initCoord[0])
N_1 = binaryEncodeLSBs(initCoord[1])
latStart = N_0    ###3161
lonStart = N_1    ###2351

x = []
y = []
latList = []
lonList = []
R_m_d1_List = []
msoList = []

### Get all possible MSO values and store all possible calculations in that list
### POSSIBLE COMBINATIONS: ((4001*[0-3200] + [0-4096]) % 3200) for both even and odd frames
### Is the only possible vision the difference between random and organized formatting?
### Can we offset the vehicles to make less collisions?
################################################################################################################################################################
###Creates 160 vehicles all in certain positions
if random:
    for i in range(numLatIncrements):
        print("Initiating latList...")
        if (i%20 == 0):
            latList.append(latStart+i)
    ###Creates a list of all possible LONGITUDE positions based on the binary encoding value of the UAT
    for i in range(numLonIncrements):
        print("Initiating lonList...")
        if (i%20 == 0):
            lonList.append(latStart+i)
else:
    for i in range(numLatIncrements):
        print("Initiating latList...")
        if (i%1 == 0):
            latList.append(latStart+i)
    ###Creates a list of all possible LONGITUDE positions based on the binary encoding value of the UAT
    for i in range(numLonIncrements):
        print("Initiating lonList...")
        if (i%1 == 0):
            lonList.append(latStart+i)

################################################################################################################################################################
initMSO = fullMSORange(lonList[0],latList[0],frameVal,randint(0,1800))
###Calculates all MSO values in the given area
print("Number of Calculations:",len(latList)*len(lonList))
print("Number of Calculations 2:",numLatIncrements*numLonIncrements)
time.sleep(3)
for n in range(numSims):
    k = 0
    R_m_d1_List = []
    for i in range(len(latList)):
        for j in range(len(lonList)):
            print("Running Simulation... Row",i,"of",numLatIncrements,"rows")
            R_m_d1_List.append(randint(0,3200))
            if sameFrame:
                currentMSO = fullMSORange(lonList[j],latList[i],frameVal,R_m_d1_List[k])
            else:
                currentMSO = fullMSORange(lonList[j],latList[i],randint(0,1800),R_m_d1_List[k])
            k += 1
            if (initMSO == currentMSO):
                x.append(lonList[j])
                y.append(latList[i])

### How does spacing affect the number of collisions?
### How much of a difference does it make to have random positions and directions versus organized positions and directions?
### How much of a difference does it make to have random frame values versus same/different frame values?
### How do we find patterns
A single MSO calculation from one vehicle depends on:
    The previous velocity
    The current velocity
    The previous position
    The current position
    The current frame value
###############################################################################################################################################################
fig = plt.figure()
ax = fig.add_subplot(111)
plt.hist2d(x,y, bins=[np.arange(N_1,N_1+numLonIncrements,10),np.arange(N_0,N_0+numLatIncrements,10)],cmap='YlOrRd')
plt.show(block=False)
print("Press enter to exit")
input("")
