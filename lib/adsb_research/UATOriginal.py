  
# from _typeshed import Self
#from lib.adsb_research.main_sim_standard_model import get_distance
from socket import NI_NUMERICSERV
import sys
sys.path.append('../')
from Vehicle import Vehicle
import numpy as np
import mso.propagation_param as p
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import csv

UAV_PHYSICAL_SIZE = 1
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

class UAT_Model:
    def __init__(self) -> None:
        self.mso = 0
        self.msos = []
        self.msoList = {}
        self.msoListSecond = []
        self.msoTotal = []
        self.visibilityList = {'overlap': [],
                                'chance': [], 
                                'closeness': [], 
                                'collisions': [], 
                                'MTL': []}
        self.overlap = 0
        self.chance = 0
        self.closeness = 0
        self.collisionsPrint = 0
        self.MTL = 0
        self.visibilityListSecond = []
        self.visibilityListTotal = []
        self.vehiclePositions = []  # Declare list of self.vehicles, empty for now
        self.vehicles = []
        self.crashes = [] #see if our avoidance didnt work and we hit another UAV
        self.totalCollisions = 0 #track total collisions for graphing later
        self.collisions = []
        self.collisionBool = 0 #So we get a bool so if a collision happened recently we can flag it.
        self.listAdded = False
        self.xPos = 0 #Regular x,y,z cartesian coordinates
        self.yPos = 0
        self.zPos = 0
        self.size = 0
        self.speed = 0
        self.maxSpeed = 0
        self.Radius = 0
        self.timeIndex = 0
        self.numSeconds = 0
        self.vehicleNum = 0
        self.num_vehicles = 0
        #self.getConst()
        #self.init_vehicles()
        # # bar = IncrementalBar("Simulating...", max=p.num_seconds)
    """We are getting the values of tf and num_vehicles from sim.yaml from params."""
    def getConst(self):
        self.num_vehicles = self.num_vehicles
        self.numSeconds = self.numSeconds
        self.init_vehicles()

    """Updating the vehicle positions, this is called in uav_sim"""
    def updateVehiclePosition(self):
        print("Is the other model getting called?")
        self.vehicles[self.vehicleNum].update_position((self.xPos, self.yPos, self.zPos))

    """This is where our vehicles are created. Called in uav_sim and in getConst here"""
    def init_vehicles(self): #check
        for i in range(self.num_vehicles):
            uas = Vehicle((0, 0, 0),random.randrange(3601), i)
            self.vehiclePositions.append([])
            self.vehicles.append(uas)
            
    """Gets the euclidean distance"""
    def get_distance(self, receiver, transmitter):
        return int(np.ceil(np.sqrt((receiver[0] - transmitter[0]) ** 2.0 +
                                  (receiver[1] - transmitter[1]) ** 2.0 +
                                  (receiver[2] - transmitter[2]) ** 2.0)))

    def evaluateMTL(self, transmitVhclNum):
        distance = self.get_distance(self.vehiclePositions[self.vehicleNum][self.timeIndex-1], self.vehiclePositions[transmitVhclNum][self.timeIndex-1])
        if distance == 0:
            distance = 1
        if distance < UAV_PHYSICAL_SIZE: #This section only checks to see if they are close 
            #enough for a physical collision. Then grabbs the values to plot them.
            #dont need this in the supercomputer.
            self.crashes.append(self.vehiclePositions[self.vehicleNum][self.timeIndex-1])
            self.crashes.append(self.vehiclePositions[transmitVhclNum][self.timeIndex-1])
        if distance < MAX_DISTANCE:
            #this grabs our values from our dictionary of powers, I think
            received_power = power_dict.get(distance)
        else:
            received_power = POWER_BELOW_MTL
        if received_power >= -90.0: 
            if random.random() > 0.99: # This is uniform distribution, we may want a gaussian.
                self.visibilityList['chance'].append(transmitVhclNum)
                self.collisionBool = 1
                self.chance += 1
                return False
        elif p.receiver_min_trigger_level <= received_power < -90.0:
            if random.random() > 0.9:
                self.collisionBool = 1
                self.visibilityList['chance'].append(transmitVhclNum)
                self.chance += 1
                return False
        else:
            self.visibilityList['MTL'].append(transmitVhclNum)
            self.MTL += 1
            return False
        return True

    def evaluateCloseness(self, transmitVhclNum, transmitMSO):
        if (self.mso - 9) <= transmitMSO <= (self.mso + 8):
            self.visibilityList['closeness'].append(transmitVhclNum)
            self.collisionBool = 1
            self.closeness += 1
            return False
        return True

    def evaluateOverlaps(self, transmitVhclNum, transmitMSO, msoList):
        #print("Mso List: ",msoList.get(transmitVhclNum - 1))
        if msoList.get(transmitVhclNum - 1) != None:
            #print("This is triggering!")
            i = transmitVhclNum - 2
            while msoList.get(transmitVhclNum - 1) != None:
                i = i - 1
            if (transmitMSO - i)%2 == 0:
                self.collisionBool = 1
                self.visibilityList['overlap'].append(transmitVhclNum)
                self.overlap += 1
                return False
            else:
                return True
        return True
        # sorted_msos = self.vehicles.copy()
        # sorted_msos.sort(key=lambda vehicle: vehicle.message_start_opportunity)  # sort vehicles by current MSO
        # previous_mso = sorted_msos[0].message_start_opportunity  # initialize previous MSO value
        # num_consecutive = 0  # initialize counter for consecutive messages
        # #print("Sorted MSO: ", len(sorted_msos))
        # for j in range(1, len(sorted_msos)):  # iterate through MSOs after first one
        #     # The first condition would be if the MSO is consecutive
        #     # The second condition matters because we don't want to reset the consecutive counter based on repeat MSOs
        #     if sorted_msos[j].message_start_opportunity == previous_mso + 1 or \
        #             sorted_msos[j].message_start_opportunity == previous_mso:
        #         #print("mso sorted: ",sorted_msos[j].message_start_opportunity)
        #         #print("Previous: ",previous_mso + 1)
        #         if sorted_msos[j].message_start_opportunity == previous_mso + 1:  # if truly consecutive
        #             num_consecutive += 1
        #         # If this MSO is consecutive and has an odd position with positions starting at 0
        #         if num_consecutive % 2:
        #             for k in range(p.num_vehicles):  # Then no vehicles receive this transmission
        #                 if sorted_msos[j].transmissions[k] == 0:  # If the transmission object exists
        #                     continue
        #                 if sorted_msos[j].transmissions[k]["successful_decode"]:
        #                     # This should modify original list because .copy() does a shallow copy
        #                     sorted_msos[j].transmissions[k]["successful_decode"] = False
        #                     #false_from_message_overlap += 1
        #                     self.overlap += 1
        #                     return False
        #     else:
        #         num_consecutive = 0
        #     previous_mso = sorted_msos[j].message_start_opportunity  # update previous MSO
        
    def evaluateCollisions(self, transmitVhclNum, transmitMSO, msoList):
        if len(msoList[transmitMSO]) > 1:
            self.visibilityList['collisions'].append(transmitVhclNum)
            self.collisionsPrint += 1
            self.collisionBool = 1
            return False
        else:
            return True
    
    def propagate(self):
        # msoList will be a dictionary
        ### Keys: MSOs
        ### Values: Vehicle Numbers

        # visibilityList will be a list of dictionaries
        ### Index: The vehicle receiving the information
        ### Dictionary
        ###### Keys: Type of collision
        ###### Values: Vehicle Numbers
        # propagateVehicle
        #vehicleNum is our current vehicle passed from uav_sim
        print("time: ",self.timeIndex)
        self.visibilityList = {'overlap': [], 'chance': [], 'closeness': [], 'collisions': [], 'MTL': []}
        self.vehiclePositions[self.vehicleNum].append((self.xPos, self.yPos, self.zPos))
        self.vehicles[self.vehicleNum].update_position(self.vehiclePositions[self.vehicleNum][self.timeIndex-1])
        if self.timeIndex == 0:
            self.vehicles[self.vehicleNum].pseudo_random_number(0)
        mso = self.vehicles[self.vehicleNum].full_mso_range()
        self.mso = mso
        self.msos.append(mso)
        if mso not in self.msoList:
            self.msoList[mso] = list()
        self.msoList[mso].append(self.vehicleNum)
        #self.evaluateOverlaps(i,transmitMSO,self.msoList)
        #print("Length msos: ", len(self.msos))
        for i in range(len(self.msos)):
            #print(self.visibilityList)
            if i == self.vehicleNum:
                i = i - 1 #So if its the first one we get the last in the list. If we 
                #are on the current vehicle.
                continue
            transmitMSO = self.msos[i] #current mso tranmitter
            if not self.evaluateMTL(i):
                i = i - 1 #if there was an MTL collision then change index and continue
                continue
            if not self.evaluateCloseness(i, transmitMSO):
                i = i - 1
                continue
            if not self.evaluateOverlaps(i, transmitMSO, self.msoList):
                i = i - 1
                continue
            if not self.evaluateCollisions(i, transmitMSO, self.msoList):
                i = i - 1
                continue
            i = i - 1
            #print("Here is i: ",i)
        self.visibilityListSecond.append(self.visibilityList)
        if self.vehicleNum == self.num_vehicles - 1:
            #Coulton
            #print("msos: ", self.msos)
            #print(self.vehiclePositions[0][self.timeIndex-1])
            #print(self.vehiclePositions[1][self.timeIndex-1])
            #print(self.vehiclePositions[2][self.timeIndex-1])
            self.visibilityListTotal.append(self.visibilityListSecond)
            collisions = self.getNumMSOCollisions()
            self.collisions.append(collisions)
            self.visibilityListSecond = []

            self.msoTotal.append(self.msoList)
            self.listAdded = True
            self.msos = []
            self.msoList = {}

        myList = self.getVisibilityList(self.visibilityList)
        if(not myList):
            print("Here")
            print(self.visibilityList)
            print("#####################")
        return myList
        
    def getVisibilityList(self, visibilityList):
        visList = []
        for key, values in visibilityList.items():
            for i in range(len(values)):
                visList.append(values[i])
        # print(visList)
        return visList

    # Returns a list because I couldn't figure out what else to do
    #Now this returns the vehicle number of the transmitting UAV. So we can see which UAV we now cannot see.
    #only returns if a collision has happened recently.
    def collisionTest(self):
        if len(self.visibilityList['collisions']) != 0 and self.collisionBool == 1:
            self.collisionBool = 0 #reset bool so only get value when a collision has recently happened
            return [1]
        elif len(self.visibilityList['overlap']) != 0 and self.collisionBool == 1: 
            #print("overlaps")
            self.collisionBool = 0 
            return [1]
        else:
            return [0]
        
    def printResults(self):
        totalMso = (p.num_seconds*self.num_vehicles * (self.num_vehicles-1))
        with open('ResultsH.csv', 'a', newline='') as csvfile: #after first run change to a
            writer = csv.writer(csvfile, delimiter=',')            
            #writer.writerow(["Num Vehicles","Watts","Collision Radius","Max Speed","Total MSO Collisions", "Phys Crash","% MSO collsions/MSO" 
            #,"Zone Violated","Total MSO","overlap","chance","closeness","collisions","MTL"])
            writer.writerow([str(self.num_vehicles),str(ERP_TYPE),str(self.Radius),str(self.maxSpeed),
           str(self.totalCollisions),str(len(self.crashes)),str(self.totalCollisions/totalMso),
                str(self.size),str(totalMso),str(self.overlap),
                    str(self.chance),str(self.closeness),
                        str(self.collisionsPrint),str(self.MTL)])
        #self.graph()
        #self.graphAni()
        print("Graph")

    def getNumMSOCollisions(self):
        collisionPairs = []
        for vehicleNum in range(len(self.visibilityListSecond)):
                for key, value in self.visibilityListSecond[vehicleNum].items():
                    if len(value) != 0:
                        for i in range(len(value)):
                            collisionPairs.append((vehicleNum,value[i]))
                            self.totalCollisions = self.totalCollisions + 1        
        return collisionPairs

        
    def graphAni(self):
        #There are way better ways to do this but unfortunately I couldnt get those others ways to work. This works but it depends on
        #the amount of vehicles we have. Anyways it works but requires work to change.
        legend_labels = []
        collisions = []
        crashesx = []
        crashesy = []
        if(self.num_vehicles != 2):
            print("Sorry! Number of vehicles is not correct in the animation.")
            return None
            
            
        positionsx1 = [self.vehiclePositions[0][0][0]]
        positionsy1 = [self.vehiclePositions[0][0][1]]
        positionsx2 = [self.vehiclePositions[1][0][0]]
        positionsy2 = [self.vehiclePositions[1][0][1]]
        # positionsx3 = [self.vehiclePositions[2][0][0]]
        # positionsy3 = [self.vehiclePositions[2][0][1]]
        # positionsx4 = [self.vehiclePositions[3][0][0]]
        # positionsy4 = [self.vehiclePositions[3][0][1]]
        # positionsx5 = [self.vehiclePositions[4][0][0]]
        # positionsy5 = [self.vehiclePositions[4][0][1]]
        # positionsx6 = [self.vehiclePositions[5][0][0]]
        # positionsy6 = [self.vehiclePositions[5][0][1]]
        # positionsx7 = [self.vehiclePositions[6][0][0]]
        # positionsy7 = [self.vehiclePositions[6][0][1]]
        # positionsx8 = [self.vehiclePositions[7][0][0]]
        # positionsy8 = [self.vehiclePositions[7][0][1]]
        # positionsx9 = [self.vehiclePositions[8][0][0]]
        # positionsy9 = [self.vehiclePositions[8][0][1]]
        # positionsx10 = [self.vehiclePositions[9][0][0]]
        # positionsy10 = [self.vehiclePositions[9][0][1]]
        plt.ion()       
        fig = plt.figure() 
        ax = fig.add_subplot(111)
        ax.set_xlim([-200, 200])
        ax.set_ylim([-200, 200])
        color = ["Red", "DarkOrange", "Yellow", "Lime", "Aqua", "DodgerBlue", "Indigo", "BlueViolet", "Fuchsia","Black"]
        li1, = ax.plot(positionsx1,positionsy1,'.',color = color[0])
        li2, = ax.plot(positionsx2,positionsy2,'.',color = color[1])
        # li3, = ax.plot(positionsx3,positionsy3,'.',color = color[2])
        # li4, = ax.plot(positionsx4,positionsy4,'.',color = color[3])
        # li5, = ax.plot(positionsx5,positionsy5,'.',color = color[4])
        # li6, = ax.plot(positionsx6,positionsy6,'.',color = color[5])
        # li7, = ax.plot(positionsx7,positionsy7,'.',color = color[6])
        # li8, = ax.plot(positionsx8,positionsy8,'.',color = color[7])
        # li9, = ax.plot(positionsx9,positionsy9,'.',color = color[8])
        # li10, = ax.plot(positionsx10,positionsy10,'.',color = color[9])
        
        fig.canvas.draw()
        plt.show(block=False)
        color_index = 0
        label_index = 0
        
        print("Printing Results")
        print("Adding waypoints")
        
        #for i in range(self.totalCollisions):
        #    legend_labels.append(legend_labels.append("Collision {}".format(i + 1)))
        print("Finish animation")
        for i in range(p.num_seconds):
            positionsx1.append(self.vehiclePositions[0][i][0])
            positionsy1.append(self.vehiclePositions[0][i][1])
            positionsx2.append(self.vehiclePositions[1][i][0])
            positionsy2.append(self.vehiclePositions[1][i][1])
            # positionsx3.append(self.vehiclePositions[2][i][0])
            # positionsy3.append(self.vehiclePositions[2][i][1])
            # positionsx4.append(self.vehiclePositions[3][i][0])
            # positionsy4.append(self.vehiclePositions[3][i][1])
            # positionsx5.append(self.vehiclePositions[4][i][0])
            # positionsy5.append(self.vehiclePositions[4][i][1])
            # positionsx6.append(self.vehiclePositions[5][i][0])
            # positionsy6.append(self.vehiclePositions[5][i][1])
            # positionsx7.append(self.vehiclePositions[6][i][0])
            # positionsy7.append(self.vehiclePositions[6][i][1])
            # positionsx8.append(self.vehiclePositions[7][i][0])
            # positionsy8.append(self.vehiclePositions[7][i][1])
            # positionsx9.append(self.vehiclePositions[8][i][0])
            # positionsy9.append(self.vehiclePositions[8][i][1])
            # positionsx10.append(self.vehiclePositions[9][i][0])
            # positionsy10.append(self.vehiclePositions[9][i][1])
            li1.set_xdata(positionsx1)
            li1.set_ydata(positionsy1)
            li2.set_xdata(positionsx2)
            li2.set_ydata(positionsy2)
            # li3.set_xdata(positionsx3)
            # li3.set_ydata(positionsy3)
            # li4.set_xdata(positionsx4)
            # li4.set_ydata(positionsy4)
            # li5.set_xdata(positionsx5)
            # li5.set_ydata(positionsy5)
            # li6.set_xdata(positionsx6)
            # li6.set_ydata(positionsy6)
            # li7.set_xdata(positionsx7)
            # li7.set_ydata(positionsy7)
            # li8.set_xdata(positionsx8)
            # li8.set_ydata(positionsy8)
            # li9.set_xdata(positionsx9)
            # li9.set_ydata(positionsy9)
            # li10.set_xdata(positionsx10)
            # li10.set_ydata(positionsy10)
            ax.autoscale_view(True,True,True)
            fig.canvas.draw()
            plt.savefig("Random{:03}.png".format(i))
            fig.canvas.flush_events()
           
            #plt.savefig(("{}V_{}fig5_main_sim." + i).format(p.num_vehicles, ERP_TYPE))
            #plt.show(block=False)
            

        # plt.savefig(("{}V_{}fig5_main_sim." + plot_type).format(p.num_vehicles, ERP_TYPE))
###############################################################################################
    def graph(self):
        legend_labels = []
        collisions = []
        crashesx = []
        crashesy = []
        positions = []
        fig, ax = plt.subplots()
        color_index = 0
        label_index = 0
        startPos = []
        endPos = []

        color = ["Red", "DarkOrange", "Yellow", "Lime", "Aqua", "DodgerBlue", "Indigo", "BlueViolet", "Fuchsia"]
        print("Printing Results")
        #print("Adding waypoints")
        #print(self.visibilityList)
        #print("second:",self.visibilityListSecond)
        #print("tot:", self.visibilityListTotal)
        for i in range(self.totalCollisions):
            #print("Do we make it here?")
            legend_labels.append(legend_labels.append("Collision {}".format(i + 1)))
        for j in range(self.num_vehicles):
            for i in range(p.num_seconds):
                if(i == 1):
                    collisions.append(ax.scatter(self.vehiclePositions[j][i][0],
                                                self.vehiclePositions[j][i][1],
                                                marker="v", c='black',s=30, label = "Start Positions"))
                                                #need to add some marker for start positions to get avg speed
                                                #But how do i know when they reached destination?
                else:
                    collisions.append(ax.scatter(self.vehiclePositions[j][i][0],
                                                self.vehiclePositions[j][i][1],
                                                marker=".", c='grey',s=2))
        
        print("Added collisions")
        for i in range(len(self.collisions)):
            for j in range(len(self.collisions[i])):
                vehicle1Coord = self.vehiclePositions[self.collisions[i][j][0]][i]
                vehicle2Coord = self.vehiclePositions[self.collisions[i][j][1]][i]
                collisions.append(ax.scatter([vehicle1Coord[0] + j,vehicle2Coord[0]],
                                                [vehicle1Coord[1],vehicle2Coord[1]+j],
                                                marker=".", c=color[color_index],     #appened ax.scatter
                                                label="Collision {}".format(label_index)))
                label_index = label_index + 1
                color_index = color_index + 1
                if color_index == 9:
                    color_index = 0

        plt.legend(collisions, legend_labels, fontsize=8)
        plt.legend(fontsize=8, bbox_to_anchor=(1, 0.7))
        plt.title("2D UAS Positions With MSO Collisions")
        ax.set_aspect("equal")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        if(len(self.crashes) > 0):
            for i in range(len(self.crashes)-1):
                    crashesy.append(self.crashes[i][1])
                    crashesx.append(self.crashes[i][0])
        ax.scatter(crashesx,crashesy,marker="x",color='red',s=20)
        #plt.savefig(("{}V_{}fig5_main_sim.").format(p.num_vehicles, ERP_TYPE))
        plt.savefig(("{}V_{}fig5_main_simAniMationCoulton.").format(self.num_vehicles, ERP_TYPE))
        plt.show()