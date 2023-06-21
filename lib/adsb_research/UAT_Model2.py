# from _typeshed import Self
from ast import keyword
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
import faulthandler
import sys
from copy import deepcopy

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
        self.num_MSO_collisions = 0  # this will represent the total number of MSO collisions
        self.vehicles = []  # This will be the list that contains all the vehicle objects and their corresponding data
        self.seconds = []  # This will be a list of lists. Each sub-list will contain the MSOs of the second
        self.vehicles_per_collision = []  # Will keep track of how many __-way MSO collisions there are
        self.collisions_per_second = []  # Contains values of number of MSO collisions in each second
        self.overall_collision_objects = []  # Contains collision objects for whole sim. These are mostly useful for plots
        self.false_from_closeness = 0  # Tallies failures from switching modes
        self.false_from_collision = 0  # Tallies failures from MSO collisions
        self.false_from_message_overlap = 0  # Tallies failures from partial overlaps
        self.false_from_minimum_trigger_level = 0  # Tallies failures from not meeting the MTL
        self.false_by_chance = 0  # Tallies random draw failures
        self.false_from_high_power = 0 
        self.seed = 0

    """We are getting the values of tf and num_vehicles from sim.yaml from params."""
    def getConst(self):
        self.num_vehicles = self.num_vehicles
        self.numSeconds = self.numSeconds
        self.seed = self.seed
        self.init_vehicles()

    def getConst(self,num_v,numS):
        self.num_vehicles = num_v
        self.numSeconds = numS
        #self.seed = self.seed
        self.init_vehicles()

    """Updating the vehicle positions, this is called in uav_sim"""
    def updateVehiclePosition(self):
        #print(self.xPos,self.yPos,self.zPos)
        #self.vehiclePositions[self.vehicleNum] = [] HERE IS A CHANGE
        self.zPos = 100.0 #set alitude to 100. 2D orientation really, just to match jonathans stuff
        self.vehicles[self.vehicleNum].update_position((float(self.yPos), float(self.xPos), float(self.zPos))) #lat and long are backwards here
        self.vehiclePositions[self.vehicleNum].append((float(self.xPos), float(self.yPos), float(self.zPos)))
        #
        #print("VehicleN Positions: ",self.vehiclePositions[self.vehicleNum])

    def updateVehiclePosition(self,j,positions):
        #print(self.xPos,self.yPos,self.zPos)
        #self.vehiclePositions[self.vehicleNum] = [] HERE IS A CHANGE
        #zPos = 100.0 #set alitude to 100. 2D orientation really, just to match jonathans stuff
        self.vehicles[j].update_position(positions) #lat and long are backwards here
        self.vehiclePositions[j].append(positions)
        #
        #print("VehicleN Positions: ",self.vehiclePositions[self.vehicleNum])

    """This is where our vehicles are created. Called in uav_sim and in getConst here"""
    def init_vehicles(self): #check
        random.seed(self.seed)
        for i in range(self.num_vehicles):
            uas = Vehicle((0, 0, 0),random.randrange(3601), i)
            self.vehiclePositions.append([]) #Change this later
            self.vehicles.append(uas)
            
    """Gets the euclidean distance"""
    def get_distance(self, receiver, transmitter):
        return float(np.ceil(np.sqrt((receiver[0] - transmitter[0]) ** 2.0 +
                                  (receiver[1] - transmitter[1]) ** 2.0 +
                                  (receiver[2] - transmitter[2]) ** 2.0)))
    
    def propagate(self):
        faulthandler.enable()
        #random.seed(self.seed)
        self.visibilityList = {'overlap': [], 'chance': [], 'closeness': [], 'collisions': [], 'MTL': []}
        #print("1")
        for j in range(self.num_vehicles):  # for each vehicle
            if self.timeIndex == 0:
                self.vehicles[j].pseudo_random_number(0)
                
            # initialize the pseudo-random number for the first second
            # This might be somewhere where the model's integrity breaks down. If we don't know when the vehicles
            # started up and what their paths were before the sim started... then we don't know what previous random
            # number should be there. I suppose for vehicles who are starting on frame zero this is accurate
        #print("2")
        # Begin MSO calculations
        self.seconds = [] #HERE IS A CHANGE
        for j in range(self.num_vehicles):  # for each vehicle
            mso = self.vehicles[j].full_mso_range()  # calculate MSO

            #print("3")
            vehicle_transmissions = []  # stores transmission dicts for each second. Will be saved for each vehicle
            # Append as many zeros as there are vehicles. In the end, if there is a zero on the list, it represents
            # the place on the list that corresponds to the vehicle storing it. So for vehicle j, the jth transmission
            # in the list will be a zero instead of a transmission dict
            for k in range(self.num_vehicles):
                vehicle_transmissions.append(0)
            #print("3.1")
            # Iterate through all of the other vehicles besides vehicle j
            for k in range(self.vehicles[j].vehicle_identifier + 1, self.vehicles[j].vehicle_identifier + self.num_vehicles):
                #print("3.2")
                distance = float(np.ceil(np.sqrt(
                    (self.vehicles[j].position[0] - self.vehicles[k % self.num_vehicles].position[0]) ** 2.0 +
                    (self.vehicles[j].position[1] - self.vehicles[k % self.num_vehicles].position[1]) ** 2.0 +
                    (self.vehicles[j].position[2] - self.vehicles[k % self.num_vehicles].position[2]) ** 2.0)))
                if distance == 0:  # Make sure there are no zero values for distance
                    distance = 1
                if distance < UAV_PHYSICAL_SIZE:
                    self.crashes.append(1)
                if distance < MAX_DISTANCE:  # if the distance is inside the max to meet MTL, get the transmit power
                    received_power = power_dict.get(distance)
                else:  # otherwise, the distance between the two vehicles is large enough that MTL is not met
                    received_power = POWER_BELOW_MTL
                meets_minimum_trigger_level = True  # initialize boolean to true
                #print("3.3")
                if received_power >= -90.0:  # Do some random draw failures
                    """For some reason, despite the same seed we get different random
                    numbers here, so we get the same results but get less chance MSOs than 
                    Jonathans direct code."""
                    free = random.random()
                    if free < 0.99:
                        meets_minimum_trigger_level = True
                    else:
                        meets_minimum_trigger_level = False
                        self.false_by_chance += 1
                        self.visibilityList['chance'].append(int(k % self.num_vehicles))
                elif p.receiver_min_trigger_level <= received_power < -90.0:  # and more random draw failures
                    if random.random() < 0.9:
                        meets_minimum_trigger_level = True
                    else:
                        meets_minimum_trigger_level = False
                        self.false_by_chance += 1
                        self.visibilityList['chance'].append(int(k % self.num_vehicles))
                else:  # Tally the MTL failures
                    meets_minimum_trigger_level = False
                    self.false_from_minimum_trigger_level += 1
                    """print("MTL")
                    print("Vehicle: ",self.vehicleNum, "Time: ",self.timeIndex)
                    print("Positions 2: ",self.vehiclePositions[j][k][0],self.vehiclePositions[j][k][1])"""
                    self.visibilityList['MTL'].append(int(k % self.num_vehicles))
                #print("3.4")
                # Create transmission object
                #####################################################################
                #Here is where our problems are, transmissions are not being created correctly, we either
                #have less than 2 transmissions in a list, which should not happen, or they are created 
                #too close together and we have too many fail due to closeness. Not sure why this is 
                #happening but I should get Jonathans veiws on this. 

                #problems happen with false_from_closeness just after this section and self.false_from_collision
                #in a later section where we are also getting too many collisions. There is also a problem as mentioned
                #before where we dont have enough transmission objects created, only one gets up in the list which causes 
                #a seg fault later as we are comparing the two tranmissions. 
                #########################################################################################3
                #SEG FAULT IS HERE, NOT SURE WHY BUT THE ALLOCATING OF THE DICTIONARY FOR SOME REASON SEG FAULTS
                transmission = {"receiverID": k % self.num_vehicles,
                                "distance": distance,
                                "receivedpower": received_power,
                                "successfuldecode": meets_minimum_trigger_level}
                #print("3.45")
                #vehicle_transmissions[transmission["receiverID"]] = transmission  # store transmission
                vehicle_transmissions[transmission["receiverID"]] = transmission
                #print("3.5")

            #self.vehicles[j].transmissions = vehicle_transmissions.copy()  # save transmission to vehicle
            self.vehicles[j].transmissions = vehicle_transmissions.copy()
            self.vehicles[j].frame += 1  # increase frame
            self.seconds.append(mso)  # record MSO in a list that helps with collisions
        # this counts failing because you're switching modes.
        # It also counts failure to decode because you're on the same MSO
        #print("4")
        for j in range(self.num_vehicles):
            for k in range(self.num_vehicles):
                # This means k corresponds to this vehicle. It's zero because you cannot transmit to yourself
                if self.vehicles[j].transmissions[k] == 0:
                    continue
                if (self.vehicles[j].message_start_opportunity - 9) <= \
                        self.vehicles[k].message_start_opportunity <= \
                        (self.vehicles[j].message_start_opportunity + 8):  # check how close MSOs are
                    if self.vehicles[j].transmissions[k]["successfuldecode"]:  # if not failed already, record failure
                        self.vehicles[j].transmissions[k]["successfuldecode"] = False
                        self.false_from_closeness += 1  # tally failure from close MSOs/switching modes
                        self.visibilityList['closeness'].append(k)
                        #This happens too many times. Need to check why this gets called too much.
        # Next, we are going to record message failures from consective MSOs
        # IMPORTANT COMMENT OUT STARTING HERE FOR MESSAGE ENHANCED MODEL
        # '''
        # Copy the vehicles list. This is likely a costly operation, and the sim could be faster and more memory
        # efficient if we did not have to do this. I think we do this instead of using seconds[i] so that the vehicles
        # themselves are sorted by MSO and we can mark message failures with greater ease
        sorted_msos = self.vehicles.copy()
        sorted_msos.sort(key=lambda vehicle: vehicle.message_start_opportunity)  # sort vehicles by current MSO
        previous_mso = sorted_msos[0].message_start_opportunity  # initialize previous MSO value
        #print("5")
        num_consecutive = 0  # initialize counter for consecutive messages
        for j in range(1, len(sorted_msos)):  # iterate through MSOs after first one
            # The first condition would be if the MSO is consecutive
            # The second condition matters because we don't want to reset the consecutive counter based on repeat MSOs
            if sorted_msos[j].message_start_opportunity == previous_mso + 1 or \
                    sorted_msos[j].message_start_opportunity == previous_mso:
                if sorted_msos[j].message_start_opportunity == previous_mso + 1:  # if truly consecutive
                    num_consecutive += 1
                # If this MSO is consecutive and has an odd position with positions starting at 0
                if num_consecutive % 2:
                    for k in range(self.num_vehicles):  # Then no vehicles receive this transmission
                        if sorted_msos[j].transmissions[k] == 0:  # If the transmission object exists
                            continue
                        if sorted_msos[j].transmissions[k]["successfuldecode"]:
                            # This should modify original list because .copy() does a shallow copy
                            sorted_msos[j].transmissions[k]["successfuldecode"] = False
                            self.false_from_message_overlap += 1
                            self.visibilityList['overlap'].append(k)
            else:
                num_consecutive = 0
            previous_mso = sorted_msos[j].message_start_opportunity  # update previous MSO
        # '''
        # IMPORTANT END SECTION TO COMMENT FOR MESSAGE ENHANCED MODEL
        # TODO missing consecutive message case of low power first message high power second message
        # Now we look at MSO collisions. First we record the collisions, then we deal with their implications
        #print("6")
        collision_objects = []  # will hold collision objects for this second
        for j in range(752, 3952):  # for each MSO
            count = self.seconds.count(j)  # count how many times that MSO occurred in the second
            if count > 1:  # if the MSO occurred more than once, there was an MSO collision
            # Figure out which vehicles
                vehicles_involved = []  # records which vehicles are involved in the collision
                vehicle_ids_involved = []
                for k in range(self.num_vehicles):  # for each vehicle
                    if self.vehicles[k].message_start_opportunity == j:  # if the MSO matches the one of the collision
                        # record the vehicle and vehicle number, which corresponds to an index in the vehicles list
                        """So problem was that this did not run and so vehicles_invloved was
                        still empty and so appended an empty list to mso_collision which 
                        caused our seg fault. Temp solution to check if vehicles involved has
                        anything in it before appending"""
                        vehicle_ids_involved.append(k)
                        vehicles_involved.append(self.vehicles[k])
                self.vehicles_per_collision.append(count)  # Record how many vehicles per collision
                # Create collision dict
                if vehicles_involved:
                    mso_collision = {"second": self.timeIndex,
                                "message_start_opportunity": j,
                                "vehicles_involved": vehicles_involved}  # record the collision as object
                
                    collision_objects.append(mso_collision)  # add the new collision to the list
        # Now we analyze implications of the MSO collisions
        #print("7")
        for j in range(len(collision_objects)):  # for every collision in this second
            mso = collision_objects[j]["message_start_opportunity"]  # get the MSO for this collision
            for k in range(self.num_vehicles):  # for every vehicle
                mso_to_check = self.vehicles[k].message_start_opportunity
                # that is not part of the collision (and not switching modes)
                if (mso_to_check < (mso - 9)) or (mso_to_check > (mso + 8)):
                    transmissions = [] #transmissions does not stay at 2 long....
                    # compare to each vehicle that was part of the collision (add them to a list)
                    for ell in range(len(collision_objects[j]["vehicles_involved"])): #collision objects are zero in seg fault
                        transmissions.append(collision_objects[j]["vehicles_involved"][ell].transmissions[k])
                        
                    # Sort the collision vehicles by distance from vehicle k
                    transmissions.sort(key=lambda transmission: transmission["distance"])
                    if transmissions[0]["receivedpower"] >= p.receiver_min_trigger_level:
                        # IMPORTANT
                        # The "3000" is the difference between this file and main_sim_collision_enhanced.py. Change the
                        # 3000 to p.power_difference_decibel to get collision enhanced model.
                        # If the power difference between the closest and second-closest vehicle is greater than or
                        # equal to the necessary power difference
                        
                        if len(transmissions) < 2: #Coulton added this, not sure if its kosher, makes sense physically though. Only one transmission so it works
                            pass
                        elif transmissions[0]["receivedpower"] - transmissions[1]["receivedpower"] >= 3000:
                            # Then the closest message succeeds and the rest do not
                            for ell in range(len(transmissions) - 1):
                                if transmissions[ell + 1]["successfuldecode"]:
                                    transmissions[ell + 1]["successfuldecode"] = False
                                    self.false_from_collision += 1
                                    self.visibilityList['collisions'].append(k)
                        # Otherwise, if the second closest doesn't meet the MTL, the closest message still succeeds
                        # because we already determined that the closest meets the MTL
                        elif transmissions[1]["receivedpower"] < p.receiver_min_trigger_level:
                            for ell in range(len(transmissions) - 1):
                                if transmissions[ell + 1]["successfuldecode"]:
                                    transmissions[ell + 1]["successfuldecode"] = False
                                    self.false_from_collision += 1
                                    self.visibilityList['collisions'].append(k)
                        # The only other option is that the closest and second-closest meet the MTL, but the difference
                        # in power is not great enough for the closest to be selected, so all messages fail
                        else:
                            for ell in range(len(transmissions)):  # make them all False
                                if transmissions[ell]["successfuldecode"]:
                                    transmissions[ell]["successfuldecode"] = False
                                    self.false_from_collision += 1
                                    self.visibilityList['collisions'].append(k)
                    # If the closest transmission doesn't meet MTL, they all fail
                    else:
                        for ell in range(len(transmissions)):  # make them all False
                            if transmissions[ell]["successfuldecode"]:
                                transmissions[ell]["successfuldecode"] = False
                                self.false_from_collision += 1
                                self.visibilityList['collisions'].append(k)
        myList = self.getVisibilityList(self.visibilityList)
        #self.graph()
        #print("8")
        return myList

    def getVisibilityList(self, visibilityList):
        visList = []
        for key, values in visibilityList.items():
            for i in range(len(values)):
                visList.append(values[i])
        final_list = list(set(visList)) #remove all duplicates
        #print("Length of final list: ", len(final_list))
        return final_list

    def collisionTest(self):
        if len(self.visibilityList['collisions']) != 0: #and self.collisionBool == 1:
            self.collisionBool = 0 #reset bool so only get value when a collision has recently happened
            return [1]
        elif len(self.visibilityList['overlap']) != 0:# and self.collisionBool == 1: 
            self.collisionBool = 0 
            return [1]
        else:
            return [0]

    def printResults(self):
        unsuccessful_decodes = self.false_from_closeness + self.false_from_collision + self.false_from_message_overlap + \
                            self.false_from_minimum_trigger_level + self.false_from_high_power + self.false_by_chance
        totalMso = (self.numSeconds*self.num_vehicles * (self.num_vehicles-1))
        self.overlap = self.false_from_message_overlap
        self.chance = self.false_by_chance
        self.MTL = self.false_from_minimum_trigger_level
        self.collisionsPrint = self.false_from_collision
        self.closeness = self.false_from_closeness
        with open('UAT500.csv', 'a', newline='') as csvfile: #after first run change to a
            writer = csv.writer(csvfile, delimiter=',')            
            #writer.writerow(["Num Vehicles","Watts","Collision Radius","Max Speed","Total MSO Collisions", "Phys Crash","% MSO collsions/MSO" 
            #,"Zone Violated","Total MSO","overlap","chance","closeness","collisions","MTL","High Power","seed"])
            writer.writerow([str(self.num_vehicles),str(ERP_TYPE),str(self.Radius),str(self.maxSpeed),
           str(self.totalCollisions),str(len(self.crashes)),str(unsuccessful_decodes/totalMso),
                str(self.size),str(totalMso),str(self.overlap),
                    str(self.chance),str(self.closeness),
                        str(self.collisionsPrint),str(self.MTL),str(self.false_from_high_power),str(self.seed)])
        #self.graph()
        #self.graphAni()

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
        for i in range(self.numSeconds):
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
           
            #plt.savefig(("{}V_{}fig5_main_sim." + i).format(self.num_vehicles, ERP_TYPE))
            #plt.show(block=False)
            

        # plt.savefig(("{}V_{}fig5_main_sim." + plot_type).format(self.num_vehicles, ERP_TYPE))
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
        #print(self.totalCollisions)
        #print(len(self.vehiclePositions))
        for i in range(self.totalCollisions):
            legend_labels.append(legend_labels.append("Collision {}".format(i + 1)))
        for j in range(self.num_vehicles):
            for i in range(0,int(self.numSeconds)):
                if(i == 1):
                    #print("Here?")
                    collisions.append(ax.scatter(self.vehiclePositions[j][i][0],
                                                self.vehiclePositions[j][i][1],
                                                marker="v", c='black',s=30, label = "Start Positions"))
                                                #need to add some marker for start positions to get avg speed
                                                #But how do i know when they reached destination?
                else:
                    #print("Or Here?")
                    collisions.append(ax.scatter(self.vehiclePositions[j][i][0],
                                                self.vehiclePositions[j][i][1],
                                                marker=".", c='grey',s=2))
                    #print("SO this is broken?")
        
        print("Added collisions")
        """for i in range(len(self.collisions)):
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
                    color_index = 0"""

        plt.legend(collisions, legend_labels, fontsize=8)
        plt.legend(fontsize=8, bbox_to_anchor=(1, 0.7))
        plt.title("2D UAS Positions With MSO Collisions")
        ax.set_aspect("equal")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        #plt.xlim([-10,110])
        #plt.ylim([-10,10])
        if(len(self.crashes) > 0):
            for i in range(len(self.crashes)-1):
                    crashesy.append(self.crashes[i][1])
                    crashesx.append(self.crashes[i][0])
        ax.scatter(crashesx,crashesy,marker="x",color='red',s=20)
        #plt.savefig(("{}V_{}fig5_main_sim.").format(self.num_vehicles, ERP_TYPE))
        #plt.savefig(("{}V_{}fig5_main_simAniMationCoulton.").format(self.num_vehicles, ERP_TYPE))
        plt.show()


# UAV = UAT_Model()
# UAV.getConst(500,100)
# for i in range(100):
#     for j in range(500):  # for each vehicle
#             UAV.updateVehiclePosition(j,p.vehicle_positions[j][i])
#     vis = UAV.propagate()
#     print("Here is i: ",i)
    