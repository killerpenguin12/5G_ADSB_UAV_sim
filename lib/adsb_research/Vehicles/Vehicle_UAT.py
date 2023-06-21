# Feb 1, 2021
# This file is the vehicle class used in normal simulation based on non gps tracks
from random import randrange
from Vehicles.Vehicle import Vehicle
from mso.UAT import UAT

# Child class to vehicle to incorporate the UAT and msos
class Vehicle_UAT(Vehicle):
    def __init__(self, lat, lon, alt, msos):
        super().__init__(lat, lon, alt)
        self.msos = msos
    
    # def __init__(self):
    #     super().__init__()
    #     self.msos = []

    def getMso(self, index):
        return self.msos[index]

    def addMso(self, mso):
        self.msos.append(mso)

    def upload_positions(self, msos, lat, long, alt):
        super().upload_positions(lat, long, alt)
        self.msos = msos
    
    def getMsos(self):
        return self.msos
        
    def resetMsos(self):
        self.msos = []









































    # # This updates the 12 L.S.B.s of the latitude and longitude
    # # TANNER CHANGE: Creates its own lcation according to 
    # def updatePosition(self, path, quadrant, masterMSO):
    #     # Test to see if the updatePosition function worked correctly
    #     # msoList = [818, 817, 816, 815, 814, 813, 812, 811, 810, 809, 808]
    #     while (True):
    #         # if msoList.__contains__(self.message_start_opportunity):
    #         #     break
    #         # print("Finding vehicle position with path {} and quadrant {}".format(path, quadrant))
    #         # print("Latitude: {} , Longitude: {}\n".format(self.latitude_lsbs, self.longitude_lsbs))
            
    #         self.position = self.newPosition(quadrant)
    #         self.latitude_lsbs, self.longitude_lsbs, altitude = self.extract_lsbs(self.position)
    #         self.full_mso_range()
    #         if self.message_start_opportunity%8 == path:# and \
    #             # (not (self.message_start_opportunity + 8 < masterMSO < self.message_start_opportunity - 9) or masterMSO == -1):
    #             break
    #     # print("\nMSO: {} , MSO%4: {} , Path: {}".format(self.message_start_opportunity, self.message_start_opportunity%4, path))

    # def update_position(self, new_position,  *args):
    #     if args:
    #         self.updatePosition(args[0], args[1], args[2])
    #     else:
    #         self.position = new_position
    #         self.latitude_lsbs, self.longitude_lsbs, altitude = self.extract_lsbs(new_position)
    #         self.full_mso_range()
    #     return self.message_start_opportunity

    # def newPosition(self, quadrant):
    #     altitude = 0
    #     latitude = 0
    #     longitude = 0
    #     if quadrant == 0:
    #         altitude = randrange(p.lowest_altitude, p.highest_altitude)
    #         latitude = randrange(0, 2*p.radius_of_area//3)
    #         longitude = randrange(0, 2*p.radius_of_area//3)
    #     elif quadrant == 1:
    #         altitude = randrange(p.lowest_altitude, p.highest_altitude)
    #         latitude = randrange(0, 2*p.radius_of_area//3)
    #         longitude = randrange(2*p.radius_of_area//3, 4*p.radius_of_area//3)
    #     elif quadrant == 2:
    #         altitude = randrange(p.lowest_altitude, p.highest_altitude)
    #         latitude = randrange(0, 2*p.radius_of_area)
    #         longitude = randrange(4*p.radius_of_area//3, 2*p.radius_of_area)
    #     elif quadrant == 3:
    #         altitude = randrange(p.lowest_altitude, p.highest_altitude)
    #         latitude = randrange(2*p.radius_of_area//3, 4*p.radius_of_area//3)
    #         longitude = randrange(0, 2*p.radius_of_area//3)
    #     elif quadrant == 4:
    #         altitude = randrange(p.lowest_altitude, p.highest_altitude)
    #         latitude = randrange(2*p.radius_of_area//3, 4*p.radius_of_area//3)
    #         longitude = randrange(2*p.radius_of_area//3, 4*p.radius_of_area//3)
    #     elif quadrant == 5:
    #         altitude = randrange(p.lowest_altitude, p.highest_altitude)
    #         latitude = randrange(2*p.radius_of_area//3, 4*p.radius_of_area//3)
    #         longitude = randrange(4*p.radius_of_area//3, 2*p.radius_of_area)
    #     elif quadrant == 6:
    #         altitude = randrange(p.lowest_altitude, p.highest_altitude)
    #         latitude = randrange(4*p.radius_of_area//3, 2*p.radius_of_area)
    #         longitude = randrange(0, 2*p.radius_of_area//3)
    #     elif quadrant == 7:
    #         altitude = randrange(p.lowest_altitude, p.highest_altitude)
    #         latitude = randrange(4*p.radius_of_area//3, 2*p.radius_of_area)
    #         longitude = randrange(2*p.radius_of_area//3, 2*p.radius_of_area)
    #     else:
    #         print("Invalid position request...")
    #         return None
        
    #     return (latitude, longitude, altitude)
