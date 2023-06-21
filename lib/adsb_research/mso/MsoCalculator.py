import mso.propagation_param as p
import numpy as np
import time
import random
import math
from progress.bar import IncrementalBar

MAX_DISTANCE = 0
POWER_BELOW_MTL = -94.0
if p.effective_radiated_power == 0.01:
    ERP_TYPE = "hundredth_watt"
    MAX_DISTANCE = 3446
    from wattValues.hundredth_watt_dict import hundredth_watt_dict as power_dict
elif p.effective_radiated_power == 0.05:
    ERP_TYPE = "twentieth_watt"
    MAX_DISTANCE = 7705
    from wattValues.twentieth_watt_dict import twentieth_watt_dict as power_dict
elif p.effective_radiated_power == 0.1:
    ERP_TYPE = "tenth_watt"
    MAX_DISTANCE = 10897
    from wattValues.tenth_watt_dict import tenth_watt_dict as power_dict
elif p.effective_radiated_power == 1.0:
    ERP_TYPE = "one_watt"
    MAX_DISTANCE = 34457
    from wattValues.one_watt_dict import one_watt_dict as power_dict
elif p.effective_radiated_power == 10.0:
    ERP_TYPE = "ten_watt"
    MAX_DISTANCE = 74081
    from wattValues.ten_watt_dict import ten_watt_dict as power_dict
elif p.effective_radiated_power == 20.0:
    ERP_TYPE = "twenty_watt"
    MAX_DISTANCE = 74081
    from wattValues.twenty_watt_dict import twenty_watt_dict as power_dict



class MsoCalculator:
    def __init__(self):
        self.false_from_high_power = []
        self.false_from_MTL = []
        self.false_by_chance = []
        self.false_from_closeness = []
        self.false_from_mso_overlap = []
        self.false_from_mso_collision = []
        self.sim = 0

        self.totalCollisions = 0
        self.oneWay = 0
        self.twoWay = 0
        self.threeWay = 0
        self.fourWay = 0
        self.fiveWay = 0
        self.sixWay = 0
        
        self.sameVehicle = "Same Vehicle"
        self.mtl = "MTL"
        self.chance = "Chance"
        self.closeness = "Closeness"
        self.overlap = "Overlap"
        self.collision = "Collision"

        self.potentialDecodes = self.initializePotentialDecodes()
        self.msos = self.initializeMSOs()
        self.initializeUnsuccessfulDecodes()

    # Had to initialize potential decodes this way in order to manipulate them correctly
    # Creates an 3D array.
    # Level 1: Represents the total number of seconds
    # Level 2: Represents each vehicle transmitting
    # Level 3: Represents each vehicle receiving 
    def initializePotentialDecodes(self):
        start = time.time()
        array = []
        for k in range(p.num_seconds):
            transmit = []
            for i in range(p.num_vehicles):
                receive = []
                for j in range(p.num_vehicles):
                    if i == j:
                        # This only happens if the same vehicle is both transmitting and receiving
                        receive.append(self.sameVehicle)
                    else:
                        receive.append(None)
                transmit.append(receive)
            array.append(transmit)
        end = time.time()
        # print("Time to initialize Potential Decodes: ", end - start)
        return array
    
    # Creates a new 2D array of msos
    # Level 1: Represents the total number of second
    # Level 2: Represents each mso value and which vehicles calculated that value
    def initializeMSOs(self):
        array = []
        for n in range(p.num_seconds):
            msos = []
            for i in range(0, 3952):
                msos.append(None)
            array.append(msos)
        return array

    # Creates an array for each simulation to sum the total decodes at the end of the program
    def initializeUnsuccessfulDecodes(self):
        for i in range(p.num_simulations):
            self.false_from_high_power.append(0)
            self.false_from_MTL.append(0)
            self.false_by_chance.append(0)
            self.false_from_closeness.append(0)
            self.false_from_mso_overlap.append(0)
            self.false_from_mso_collision.append(0)
        
    # Adds a vehicle to the mso slot that calculated that particular mso in the given second
    def addMsos(self, vehicles):
        for n in range(p.num_seconds):
            for vehicleNum in range(p.num_vehicles):
                vehicleMso = vehicles[vehicleNum].getMso(n)
                if self.msos[n][vehicleMso] == None:
                    self.msos[n][vehicleMso] = [vehicleNum]
                else:
                    self.msos[n][vehicleMso].append(vehicleNum)

    def get_distance(self, transmitCoord, receivingCoord):
        return np.int(np.ceil(np.sqrt((transmitCoord[0] - receivingCoord[0]) ** 2.0 +
                                  (transmitCoord[1] - receivingCoord[1]) ** 2.0 +
                                  (transmitCoord[2] - receivingCoord[2]) ** 2.0)))

    # Goes through each simulation and adds up all of the unsuccessful decodes
    def evaluateAllMSOs(self, vehicles):
        bar = IncrementalBar("Simulating...", max=p.num_seconds)
        for n in range(p.num_seconds):
            # if p.high_power:
            #     self.evaluateHighPower(vehicles, n)
            if p.MTL is not False or p.chance is not False:
                self.evaluateMTL(vehicles, n)
            self.evaluateMsoSubsets(vehicles, n)
            self.calculateCollisions()
            
            bar.next()
        bar.finish()
    
    # Evaluates Minimum Trigger Level and Chance
    def evaluateMTL(self, vehicles, second):
        for transmitVehicle in range(p.num_vehicles):
            for receivingVehicle in range(p.num_vehicles):
                if self.potentialDecodes[second][transmitVehicle][receivingVehicle] == None:
                    distance = self.get_distance(vehicles[transmitVehicle].getCoordinates(second), \
                        vehicles[receivingVehicle].getCoordinates(second))
                    if distance == 0:
                        distance = 1
                    if distance < MAX_DISTANCE:
                        received_power = power_dict.get(distance)
                    else:
                        received_power = POWER_BELOW_MTL

                    if received_power >= -90.0:
                        if p.chance:
                            if random.random() >= 0.99:
                                self.potentialDecodes[second][transmitVehicle][receivingVehicle] = self.chance
                                self.false_by_chance[self.sim] += 1
                    elif p.receiver_min_trigger_level <= received_power < -90.0:
                        if p.chance:
                            if random.random() >= 0.9:
                                self.potentialDecodes[second][transmitVehicle][receivingVehicle] = self.chance
                                self.false_by_chance[self.sim] += 1
                    else:
                        if p.MTL:
                            self.potentialDecodes[second][transmitVehicle][receivingVehicle] = self.mtl
                            self.false_from_MTL[self.sim] += 1
        
    # Evaluates Closeness, Overlap and Collisions in that order
    def evaluateMsoSubsets(self, vehicles, second):
        for mso in range(752, 3952):
            if self.msos[second][mso] != None:
                for j in range(len(self.msos[second][mso])):
                    if p.closeness:
                        self.evaluateCloseness(self.msos[second][mso][j], second, mso)
        if p.overlap:
            self.evaluateOverlap(second)
        self.evaluateCollisions(vehicles, second)

    ### QUESTION: Which vehicle is transmitting and which vehicle is receiving for the -9 and +8 differences? 
    # This doesn't make a huge impact (only one mso difference) but it would be nice to know
    def evaluateCloseness(self, transmitVehicle, second, currentMsoValue):
        for mso in range(currentMsoValue - 9, currentMsoValue + 8):
            if mso < 752 or mso > 3951:
                continue
            else:
                if self.msos[second][mso] != None:
                    for j in range(len(self.msos[second][mso])):
                        if transmitVehicle != self.msos[second][mso][j]:
                            if self.potentialDecodes[second][transmitVehicle][self.msos[second][mso][j]] == None:
                                self.potentialDecodes[second][transmitVehicle][self.msos[second][mso][j]] = self.closeness
                                self.false_from_closeness[self.sim] += 1
    
    # Evaluates Overlap, does not include three consecutive msos, has to be 2 in a row and then two more
    def evaluateOverlap(self, second):
        for receivingVehicle in range(p.num_vehicles):
            for mso in range(753, 3951):
                if self.msos[second][mso] != None:
                    for k in range(len(self.msos[second][mso])):
                        transmitVehicle = self.msos[second][mso][k]
                        if self.msos[second][mso-1] != None and self.potentialDecodes[second][self.msos[second][mso-1][0]][receivingVehicle] == None and receivingVehicle != transmitVehicle:
                            if self.potentialDecodes[second][transmitVehicle][receivingVehicle] == None:
                                self.potentialDecodes[second][transmitVehicle][receivingVehicle] = self.overlap
                                self.false_from_mso_overlap[self.sim] += 1

    # Evaluates all collisions
    def evaluateCollisions(self, vehicles, second):
        for receivingVehicle in range(p.num_vehicles):
            for mso in range(753, 3951):
                if self.msos[second][mso] != None and len(self.msos[second][mso]) > 1:
                    for k in range(len(self.msos[second][mso])):
                        transmitVehicle = self.msos[second][mso][k]
                        if self.potentialDecodes[second][transmitVehicle][receivingVehicle] == None:
                            self.potentialDecodes[second][transmitVehicle][receivingVehicle] = self.collision
                            self.false_from_mso_collision[self.sim] += 1
    
    # Restarts initializations and sets the arrays to the correct simulation
    def getSimulation(self, sim):
        self.potentialDecodes = self.initializePotentialDecodes()
        self.msos = self.initializeMSOs()
        self.sim = sim

    def calculateCollisions(self):
        self.totalCollisions = 0
        self.twoWay = 0
        self.threeWay = 0
        self.fourWay = 0
        self.fiveWay = 0
        self.sixWay = 0
        for n in range(p.num_seconds):
            for i in range(len(self.msos[n])):
                if self.msos[n][i] != None:
                    if len(self.msos[n][i]) < 2:
                        pass
                    elif len(self.msos[n][i]) == 2:
                        self.twoWay += 1
                        self.totalCollisions += 1
                    elif len(self.msos[n][i]) == 3:
                        self.threeWay += 1
                        self.totalCollisions += 2
                    elif len(self.msos[n][i]) == 4:
                        self.fourWay += 1
                        self.totalCollisions += 6
                    elif len(self.msos[n][i]) == 5:
                        self.fiveWay += 1
                        self.totalCollisions += 10
                    elif len(self.msos[n][i]) == 6:
                        self.sixWay += 1
                        self.totalCollisions += 15
                    # elif len(self.msos[n][i]) > p.num_vehicles//2:
                    #     self.totalCollisions = np.inf
                    else:
                        self.totalCollisions += self.combo(len(self.msos[n][i]),2)                            
    def combo(self, n, z):
        nMinZ = math.factorial(n - z)
        n = math.factorial(n)
        z = math.factorial(z)
        return n/(z*nMinZ)

    # Evaluates all simulations for potential decode and presents the information here
    def performCalculations(self, i):
        # false_from_MTL = 0
        # false_by_chance = 0
        # false_from_closeness = 0
        # false_from_overlap = 0
        # false_from_collision = 0
        false_from_high_power = 0
        # for i in range(p.num_simulations):
        #     false_from_MTL += self.false_from_MTL[i]
        #     false_by_chance += self.false_by_chance[i]
        #     false_from_closeness += self.false_from_closeness[i]
        #     false_from_overlap += self.false_from_mso_overlap[i]
        #     false_from_collision += self.false_from_mso_collision[i]
        false_from_MTL = self.false_from_MTL[i]
        false_by_chance = self.false_by_chance[i]
        false_from_closeness = self.false_from_closeness[i]
        false_from_overlap = self.false_from_mso_overlap[i]
        false_from_collision = self.false_from_mso_collision[i]
        print("\n\033[1mNumber of Simulations:\033[0m ", p.num_simulations)
        print("\033[1mNumber of Seconds per Sim:\033[0m ", p.num_seconds)
        print("\033[1mNumber of Vehicles:\033[0m ", p.num_vehicles)
        print("\033[1mPower:\033[0m ", p.effective_radiated_power, "W")
        print("\033[1mPath:\033[0m ")
        print("\033[1mSame Frame Value:\033[0m ", p.sameFrame)
        print("\033[1mDynamics:\033[0m Simulated and non realistic")
        print("-------------------------------------------------------------------------------------")
        unsuccessfulDecodes = false_from_closeness + false_from_MTL + false_from_overlap + false_from_collision \
            + false_by_chance + false_from_high_power
        if p.mtl_ignored:
            unsuccessfulDecodes = unsuccessfulDecodes - false_from_MTL
        print("\033[1mNumber of Unsuccessful to Potential Decodes:\033[0m ", unsuccessfulDecodes, "/", p.num_desired_decodes)
        print("\033[1mPercentage of Unsuccessful to Potential Decodes:\033[0m ", round((unsuccessfulDecodes/(p.num_desired_decodes))*100,3), "%")
        if unsuccessfulDecodes == 0:
            unsuccessfulDecodes = 1
        # if p.high_power:
        #     print("High Power: ", (self.false_from_high_power/unsuccessfulDecodes)*100, "%")
        # else:
        #     print("High Power: --")
        if p.MTL:
            print("\033[1mMTL:\033[0m ", round((false_from_MTL/(unsuccessfulDecodes+false_from_MTL))*100,3), "%")
        else:
            print("MTL: --")
        if p.chance:
            print("\033[1mChance:\033[0m ", round((false_by_chance/unsuccessfulDecodes)*100,3), "%")
        else:
            print("Chance: --")
        if p.closeness:
             print("\033[1mCloseness:\033[0m ", round((false_from_closeness/unsuccessfulDecodes)*100,3), "%")
        else:
            print("Closeness: --")
        if p.overlap:
            print("\033[1mOverlap:\033[0m ", round((false_from_overlap/unsuccessfulDecodes)*100,3), "%")
        else:
            print("Overlap: --")
        print("\033[1mCollision:\033[0m ", round((false_from_collision/unsuccessfulDecodes)*100,3), "%")

        print("Total Collisions: ", self.totalCollisions)
        print("Two-way Collisions: ", self.twoWay)
        print("Three-way Collisions: ", self.threeWay)
        if self.fourWay != 0:
            print("Four-way Collisions: ", self.fourWay)
        if self.fiveWay != 0:
            print("Five-way Collisions: ", self.fiveWay)
        if self.sixWay != 0:
            print("Six-way Collisions: ", self.sixWay)
    
