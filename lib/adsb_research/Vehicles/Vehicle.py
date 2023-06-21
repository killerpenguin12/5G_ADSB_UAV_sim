# Feb 1, 2021
# This file is the vehicle class used in normal simulation based on non gps tracks

import numpy as np
import pymap3d as pymap3d
import mso.propagation_param as p
from dec2bin import dec2bin
from random import randrange
from mso.UAT import UAT

# Parent vehicle class so it can be used in another research topic if needed
class Vehicle:
    def __init__(self, lat, lon, alt):
        self.latitude = lat  # 12 L.S.B.'s of the most recent valid "LATITUDE"
        self.longitude = lon  # 12 L.S.B.'s of the most recent valid "LONGITUDE"
        self.altitude = alt
    
    # def __init__(self) -> None:
    #     self.latitude = []
    #     self.longitude = []
    #     self.altitude = []

    def update_position(self, lat, long, alt):
        self.latitude.append(lat)
        self.longitude.append(long)
        self.altitude.append(alt)

    def upload_positions(self, lat, long, alt):
        self.latitude = lat
        self.longitude = long
        self.altitude = alt
    
    def getLatitude(self, second):
        return self.latitude[second]

    def getLongitude(self, second):
        return self.longitude[second]

    def getAltitude(self, second):
        return self.altitude[second]

    def getLatitudes(self):
        return self.latitude

    def getLongitudes(self):
        return self.longitude

    def getAltitudes(self):
        return self.altitude

    def getCoordinates(self, second):
        return self.latitude[second], self.longitude[second], self.altitude[second]
        
    def getAllCoordinates(self):
        return self.latitude, self.longitude, self.altitude