'''
This file is the vehicle class used in simulation
'''
import numpy as np
import pymap3d as pymap3d
import propagationParam as p
from dec2bin import dec2bin
from random import seed
from random import randint
LSB = 360./(2.**24.)
binaryDigitCycle = 4096

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
def pseudoRandomNumberR(long,lat,m):
    R_m_d1 = 1
    if (m%2 == 1):
        lsbVal = long
    else:
        lsbVal = lat
    if m == 0:
        R_m_d1 = (lat % 3200) # calculate pseudo-random number
        return R_m_d1
    elif m >= 1:
        R_m_d1 = ((4001*R_m_d1 + lsbVal%binaryDigitCycle) % 3200) # calculate pseudo-random number
        # R_m_d1 = ((4001*R_m_d1 + leastSignificantBits(m % 2)) % 3200) # calculate pseudo-random number
        return R_m_d1
    else:
        print('ERROR: m is invalid in R(m)')

################################################################################################################################################################
def fullMSORange(long,lat,m):
    MSO = (752 + pseudoRandomNumberR(long,lat,m))   ###DO WE EVEN NEED THIS?????
    return MSO
