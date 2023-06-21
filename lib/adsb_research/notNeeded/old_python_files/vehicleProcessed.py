'''
File no longer in use 5/5/2020
'''
import numpy as np

# The idea with this version of the class is that
# we can have all of the data on and individual vehicle
# beforehand--MSOs, positions, etc.
# Then in a main file we can analyze how that data interacts and draw conclusions

class vehicle:
    def __init__(self, positions):
        self.vehicle_positions = positions.copy()
        self.LAT_LSBs = []
        self.LON_LSBs = []
        self.individual_MSOs = [] # list for recording all of the MSOs
        self.frames = []
        self.randomNumbers = []
        self.k = 0 # the frame just prior to entering the restricted MSO range mode

        for i in range(len(self.vehicle_positions)): # current values may be pointless?
            self.LAT_LSBs[i] = getLSBs(self.vehicle_positions[i][0])
            self.LON_LSBs[i] = getLSBs(self.vehicle_positions[i][1])




    # Returns the 12 L.S.B.'s of the most recent valid "LATITUDE" or
    # "LONGITUDE", based on the value of m
    # DO-282B pg. 84-85
    def leastSignificantBits(self, m):
        if m == 0:
            return self.LAT_LSBs[self.m] # latitude
        elif m == 1:
            return self.LON_LSBs[self.m] # longitude
        else:
            print('ERROR: m is invalid in N(0) or N(1)')

    # Returns a pseudo-random number
    # (In order to get R(m -1), this function cannot be used,
    # since R(m - 1) is the previously chosen random number.
    # If the vehicle latitude or longitude has changed, this
    # function may return a different value than the previous
    # random number) DO-282B pg. 84-85
    def pseudoRandomNumberR(self, m):
        if m == 0:
            self.R_m_d1 = (self.leastSignificantBits(0) % 3200) # calculate pseudo-random number
            return self.R_m_d1
        elif m >= 1:
            self.R_m_d1 = ((4001*self.R_m_d1 + self.leastSignificantBits(m % 2)) % 3200) # calculate pseudo-random number
            return self.R_m_d1
        else:
            print('ERROR: m is invalid in R(m)')

    # See MSO details on pdf page 115 of of DO-B282 (page 85)
    # Returns the initial MSO when it is first being calculated
    # DO-282B pg. 84-85
    def fullMSORange(self):
        self.k = self.m # update k
        self.MSO = (752 + self.pseudoRandomNumberR(self.m)) # calculate MSO
        self.individual_MSOs.append(self.MSO) # record MSO
        return self.MSO # return MSO

    # Returns an MSO once the vehicle is no longer in the full MSO range mode
    # DO-282B pg. 84-85
    def restrictedMSORange(self):
        R_star = self.pseudoRandomNumberR(self.k) - (self.pseudoRandomNumberR(self.k) % 800) # definition of R_star (R*)
        self.MSO = (752 + R_star + (self.pseudoRandomNumberR(self.m) % 800)) # calculate MSO
        self.individual_MSOs.append(self.MSO) # record MSO
        return self.MSO # return MSO

    # This updates the 12 L.S.B.s of the latitude and longitude
    def updateLatLon(self, N_0, N_1):
        self.N_0 = N_0
        self.N_1 = N_1
        #self.LAT.append(N_0)
        #self.LON.append(N_1)

    def getLSBs(self value):
        return #somehow take the 12 LSBs
