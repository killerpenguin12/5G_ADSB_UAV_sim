# Jonathan Barrett
# Dec 2, 2021
# This is the main model simulation file. I am trying to make a decently clean file here.
"""
To prepare to run this code:
- Install all the necessary packages
- Verify the parameters of the section from propagation_param.py that will give positions
"""

import time
import numpy as np
import mso.propagation_param as p
import random
from Vehicle import Vehicle
import matplotlib.pyplot as plt  # this is used only if the plotting section is not commented out
from progress.bar import IncrementalBar
import csv
from datetime import datetime


MAX_DISTANCE = None  # declare variable
POWER_BELOW_MTL = -94.0  # this value is used as one that is distinctly below the MTL
# Based on the value of p.effective_radiated_power we import a dictionary of power values and set a maximum distance
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


if __name__ == "__main__":
    start = time.time()  # optional. Used for timing how fast the simulation runs
    #We will do numvehicles and power in order
    num_MSO_collisions = 0  # this will represent the total number of MSO collisions
    vehicles = []  # This will be the list that contains all the vehicle objects and their corresponding data
    seconds = []  # This will be a list of lists. Each sub-list will contain the MSOs of the second
    vehicles_per_collision = []  # Will keep track of how many __-way MSO collisions there are
    collisions_per_second = []  # Contains values of number of MSO collisions in each second
    overall_collision_objects = []  # Contains collision objects for whole sim. These are mostly useful for plots
    false_from_closeness = 0  # Tallies failures from switching modes
    false_from_collision = 0  # Tallies failures from MSO collisions
    false_from_message_overlap = 0  # Tallies failures from partial overlaps
    false_from_minimum_trigger_level = 0  # Tallies failures from not meeting the MTL
    false_by_chance = 0  # Tallies random draw failures
    false_from_high_power = 0  # Tallies failures from power being too high... I am not sure this ever matters
    
    #num_vehicles = sys.argv[1]

    #bar = IncrementalBar("Simulating...", max=p.num_seconds)  # Initialize progress bar
    # Populate vehicles list with vehicles
    #random.seed(5)
    seed = datetime.now()
    random.seed(seed)
    for i in range(p.num_vehicles):
        # Initial position comes in elsewhere, the random val is a random frame number for no runtime up to an hour of
        # runtime
        #random.seed(0)
        #
        uas = Vehicle((0, 0, 0), random.randrange(3601), i)  # create object
        #uas = Vehicle((0, 0, 0), 0, i)
        vehicles.append(uas)  # store object
    # Now we begin to iterate through every second of the simulation
    for i in range(p.num_seconds):
        #print("Time: ",i)
        # std::ofstream pos("Positions.csv" , std::ios::app );
        #   pos << i << "," << coord_dist_1[0] << "," << coord_dist_1[1] << '\n';
        #   pos.close();
        
        count_coulton = 0
        #print("##############################################")
        msos_per_second = []
        seconds.append(msos_per_second)  # Puts a list of MSOs in for every second in seconds array
        # Before calculating MSOs, update the vehicle positions
        #print("#######################################3")
        #print("Len vehicle Positions: ",len(p.vehicle_positions))
        for j in range(p.num_vehicles):  # for each vehicle
            #print(len(p.vehicle_positions[j]))
            #print(p.vehicle_positions[j][20])
            #print("i: ",i)
            vehicles[j].update_position(p.vehicle_positions[j][i])  # update the position of the vehicle
            #print("Positions: ",p.vehicle_positions[j][i])
            # print("Time: ",i)
            with open('Positions.csv', 'a', newline='') as csvfile: #after first run change to a
                                writer = csv.writer(csvfile, delimiter=',')            
                                writer.writerow([str(j),str(vehicles[j].position[0]),str(vehicles[j].position[1])])
            # initialize the pseudo-random number for the first second
            # This might be somewhere where the model's integrity breaks down. If we don't know when the vehicles
            # started up and what their paths were before the sim started... then we don't know what previous random
            # number should be there. I suppose for vehicles who are starting on frame zero this is accurate
            if i == 0:
                vehicles[j].pseudo_random_number(0)
        # Begin MSO calculations
        # print("second: ",i)
        
        for j in range(p.num_vehicles):  # for each vehicle
            #count_coulton += 1
            #print("Time: ",i)
            mso = vehicles[j].full_mso_range()  # calculate MSO
            with open('MSO.csv', 'a', newline='') as csvfile: #after first run change to a
                                writer = csv.writer(csvfile, delimiter=',')            
                                writer.writerow([str(i),str(mso),str(j)])
            #print("##############################################")
            #print("MSO: ",mso," Vehicle: ",j)
            #print("lat lsbs: ",vehicles[j].latitude_lsbs," Long lsbs: ",vehicles[j].latitude_lsbs)
            #print("Lat, Long: ", vehicles[j].get_lat_lon_alt(p.vehicle_positions[j][i]))
            #print("Vehicle Positions",p.vehicle_positions[j][i])
            #print("##############################################")
            
            vehicle_transmissions = []  # stores transmission dicts for each second. Will be saved for each vehicle
            # Append as many zeros as there are vehicles. In the end, if there is a zero on the list, it represents
            # the place on the list that corresponds to the vehicle storing it. So for vehicle j, the jth transmission
            # in the list will be a zero instead of a transmission dict
            for k in range(p.num_vehicles):
                vehicle_transmissions.append(0)
                #count += 1
            # Iterate through all of the other vehicles besides vehicle j
            # vel_count = 0
            for k in range(vehicles[j].vehicle_identifier + 1, vehicles[j].vehicle_identifier + p.num_vehicles):
                # calculate the distance between vehicle j and vehicle k. Round up to next integer
                #print("k: ",k)
                #print("Vehicle Identifier: ",vehicles[j].vehicle_identifier)
                #print("j: ", j)
                #print("k %: ",k % 20)
                # vel_count += 1
                # print("Current Vel: ",j)
                # print("Vel Count: ",vel_count)
                distance = int(np.ceil(np.sqrt(
                    (vehicles[j].position[0] - vehicles[k % p.num_vehicles].position[0]) ** 2.0 +
                    (vehicles[j].position[1] - vehicles[k % p.num_vehicles].position[1]) ** 2.0 +
                    (vehicles[j].position[2] - vehicles[k % p.num_vehicles].position[2]) ** 2.0)))
                
                # print("Position1: ",vehicles[j].position[0],",",vehicles[j].position[1])
                # print("Position2: ",vehicles[k % p.num_vehicles].position[0],",",vehicles[k % p.num_vehicles].position[1])
                # print("Other vehicle number: ",(k % p.num_vehicles))
                
                if distance == 0:  # Make sure there are no zero values for distance
                    distance = 1
                if distance < MAX_DISTANCE:  # if the distance is inside the max to meet MTL, get the transmit power
                    received_power = power_dict.get(distance)
                    # print("distance: ",distance)
                    # print("power: ",received_power)
                else:  # otherwise, the distance between the two vehicles is large enough that MTL is not met
                    received_power = POWER_BELOW_MTL
                    
                meets_minimum_trigger_level = True  # initialize boolean to true
                if received_power >= -90.0:  # Do some random draw failures
                    #print("Happened! ",i)

                    free = random.random()
                    #print("Random_before: ",free)
                    if free < 0.99:
                        meets_minimum_trigger_level = True
                    else:
                        #print("random: ",free)
                        meets_minimum_trigger_level = False
                        false_by_chance += 1
                        #print("Chance")
                        #print("Vehicle: ",j, "Time: ",i)
                        #print("Positions: ",p.vehicle_positions[j][i])
                        #print("First: ")
                        #print("Vehicle_num: ",j)
                        #print("Time_Index: ", i)
                elif p.receiver_min_trigger_level <= received_power < -90.0:  # and more random draw failures
                    if random.random() < 0.9:
                        meets_minimum_trigger_level = True
                    else:
                        meets_minimum_trigger_level = False
                        false_by_chance += 1
                        #print("Chance")
                        #print("Vehicle: ",j, "Time: ",i)
                        #print("Positions: ",p.vehicle_positions[j][i])
                        #print("Second: ")
                        #print("Vehicle_num: ",j)
                        #print("Time_Index: ", i)
                else:  # Tally the MTL failures
                    meets_minimum_trigger_level = False
                    false_from_minimum_trigger_level += 1
                    #print("MTL")
                    #print("Vehicle: ",j, "Time: ",i)
                    #print("Positions: ",p.vehicle_positions[j][i])
                # Create transmission object
                #####################################################################333
                ########Same until here
                #print("here")
                #print("Distance ",j,":",distance,", Transmission ID: ",(k % p.num_vehicles))
                count_coulton += 1
                transmission = {"receiver_ID": vehicles[k % p.num_vehicles].vehicle_identifier,
                                "distance": distance,
                                "received_power": received_power,
                                "successful_decode": meets_minimum_trigger_level}
                vehicle_transmissions[transmission["receiver_ID"]] = transmission  # store transmission
            vehicles[j].transmissions = vehicle_transmissions.copy()  # save transmission to vehicle
            #print("Coulton Count: ",count_coulton)
            vehicles[j].frame += 1  # increase frame
            seconds[i].append(mso)  # record MSO in a list that helps with collisions
        for j in range(p.num_vehicles):
            for k in range(p.num_vehicles):
                # This means k corresponds to this vehicle. It's zero because you cannot transmit to yourself
                #print("mso 1: ",vehicles[j].message_start_opportunity)
                #print("mso 2: ",vehicles[k].message_start_opportunity)
                if vehicles[j].transmissions[k] == 0:
                    continue
                #if(k == j):
                    #print("same")
                if (vehicles[j].message_start_opportunity - 9) <= \
                        vehicles[k].message_start_opportunity <= \
                        (vehicles[j].message_start_opportunity + 8):  # check how close MSOs are
                    if vehicles[j].transmissions[k]["successful_decode"]:  # if not failed already, record failure
                        vehicles[j].transmissions[k]["successful_decode"] = False
                        false_from_closeness += 1  # tally failure from close MSOs/switching modes
                        with open('Collisions.csv', 'a', newline='') as csvfile: #after first run change to a
                                writer = csv.writer(csvfile, delimiter=',')            
                                writer.writerow([str(j),str(k),str(vehicles[j].position[0]),str(vehicles[j].position[1]),str(vehicles[k].position[0]),str(vehicles[k].position[1]),str(i),"Closeness"])
        # Next, we are going to record message failures from consective MSOs
        # IMPORTANT COMMENT OUT STARTING HERE FOR MESSAGE ENHANCED MODEL
        # '''
        # Copy the vehicles list. This is likely a costly operation, and the sim could be faster and more memory
        # efficient if we did not have to do this. I think we do this instead of using seconds[i] so that the vehicles
        # themselves are sorted by MSO and we can mark message failures with greater ease
        sorted_msos = vehicles.copy()
        #print("Sorted length: ",len(sorted_msos))
        #print("############################################")
        sorted_msos.sort(key=lambda vehicle: vehicle.message_start_opportunity)  # sort vehicles by current MSO
        # for j in range(0,len(sorted_msos)):
        #      print(" msos: ",sorted_msos[j].message_start_opportunity, " Second: ",sorted_msos[j].vehicle_identifier)
        # print("############################################")
        previous_mso = sorted_msos[0].message_start_opportunity  # initialize previous MSO value
        num_consecutive = 0  # initialize counter for consecutive messages
        #print("Sorted MSO: ", len(sorted_msos))
        for j in range(1, len(sorted_msos)):  # iterate through MSOs after first one
            # The first condition would be if the MSO is consecutive
            #print("iteration number: ",j)
            # The second condition matters because we don't want to reset the consecutive counter based on repeat MSOs
            if sorted_msos[j].message_start_opportunity == previous_mso + 1 or \
                    sorted_msos[j].message_start_opportunity == previous_mso:
                #print("mso sorted: ",sorted_msos[j].message_start_opportunity)
                #print("Previous: ",previous_mso + 1)
                if sorted_msos[j].message_start_opportunity == previous_mso + 1:  # if truly consecutive
                    num_consecutive += 1
                    #print("first: ",sorted_msos[j].message_start_opportunity," previous: ",previous_mso)
                # If this MSO is consecutive and has an odd position with positions starting at 0
                if num_consecutive % 2:
                    for k in range(p.num_vehicles):  # Then no vehicles receive this transmission
                        if sorted_msos[j].transmissions[k] == 0:  # If the transmission object exists
                            continue
                        if sorted_msos[j].transmissions[k]["successful_decode"]:
                            # This should modify original list because .copy() does a shallow copy
                            sorted_msos[j].transmissions[k]["successful_decode"] = False
                            false_from_message_overlap += 1
                            with open('Collisions.csv', 'a', newline='') as csvfile: #after first run change to a
                                writer = csv.writer(csvfile, delimiter=',')            
                                writer.writerow([str(j),str(k),str(vehicles[j].position[0]),str(vehicles[j].position[1]),str(vehicles[k].position[0]),str(vehicles[k].position[1]),str(i),"Overlap"])
                                # std::ofstream pos("Collisions.csv" , std::ios::app );
                                # pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << "," << t << '\n';
                                # pos.close();

            else:
                num_consecutive = 0
            previous_mso = sorted_msos[j].message_start_opportunity  # update previous MSO
        # '''
        count_success = 0
        # IMPORTANT END SECTION TO COMMENT FOR MESSAGE ENHANCED MODEL
        # TODO missing consecutive message case of low power first message high power second message
        # Now we look at MSO collisions. First we record the collisions, then we deal with their implications
        collision_objects = []  # will hold collision objects for this second
        for j in range(752, 3952):  # for each MSO
            count = seconds[i].count(j)  # count how many times that MSO occurred in the second
            
            #print("Count: ",count)
            if count > 1:  # if the MSO occurred more than once, there was an MSO collision
                # Figure out which vehicles
                #print("Count greater than one")
                #print("seconds[i]: ",seconds[i])
                vehicles_involved = []  # records which vehicles are involved in the collision
                vehicle_ids_involved = []
                for k in range(p.num_vehicles):  # for each vehicle
                    if vehicles[k].message_start_opportunity == j:  # if the MSO matches the one of the collision
                        # record the vehicle and vehicle number, which corresponds to an index in the vehicles list
                        vehicle_ids_involved.append(k)
                        #print("It matches!")
                        # print("")
                        # print("Vehicles Involved: ",k,vehicles[k].transmissions)
                        # print("")
                        vehicles_involved.append(vehicles[k])
                vehicles_per_collision.append(count)  # Record how many vehicles per collision
                # Create collision dict
                # print("Collision Objects mso: ",j)
                mso_collision = {"second": i,
                                 "message_start_opportunity": j,
                                 "vehicles_involved": vehicles_involved}  # record the collision as object
                collision_objects.append(mso_collision)  # add the new collision to the list

        for j in range(len(collision_objects)):  # for every collision in this second
            mso = collision_objects[j]["message_start_opportunity"]  # get the MSO for this collision
            for k in range(p.num_vehicles):  # for every vehicle
                mso_to_check = vehicles[k].message_start_opportunity
                # that is not part of the collision (and not switching modes)
                #print("Collision MSO: ",mso)
                # print("Mso to check: ",mso_to_check)
                if (mso_to_check < (mso - 9)) or (mso_to_check > (mso + 8)):
                    transmissions = []
                    # for ell in range(len(vehicles)):
                    #     for ill in range(len(vehicles[ell].transmissions)):
                    #         print("index 1: ",ell," index 2: ",ill, " Transmission: ",vehicles[ell].transmissions[ill])#" reciever index: ",vehicles[ell].transmissions[ill])
                            #print(str(vehicles[ell].transmissions[ill]))
                            #vehicles[j].transmissions[k]["successful_decode"]
                    # compare to each vehicle that was part of the collision (add them to a list)
                    # print("Collision Objects size: ", len(collision_objects[j]["vehicles_involved"]))
                    for ell in range(len(collision_objects[j]["vehicles_involved"])):
                        transmissions.append(collision_objects[j]["vehicles_involved"][ell].transmissions[k])

                    transmissions.sort(key=lambda transmission: transmission["distance"])
                    # If the transmission from the closest vehicle meets the MTL
                    # for kk in range(len(transmissions)):
                    #    print("Transmissions: ",transmissions[kk])
                    if transmissions[0]["received_power"] >= p.receiver_min_trigger_level:
                        # IMPORTANT
                        # The "3000" is the difference between this file and main_sim_collision_enhanced.py. Change the
                        # 3000 to p.power_difference_decibel to get collision enhanced model.
                        # If the power difference between the closest and second-closest vehicle is greater than or
                        # equal to the necessary power difference
                        if transmissions[0]["received_power"] - transmissions[1]["received_power"] >= 3000:
                            # Then the closest message succeeds and the rest do not
                            #print("First one")
                            for ell in range(len(transmissions) - 1):
                                if transmissions[ell + 1]["successful_decode"]:
                                    transmissions[ell + 1]["successful_decode"] = False
                                    false_from_collision += 1
                                    with open('Collisions.csv', 'a', newline='') as csvfile: #after first run change to a
                                        writer = csv.writer(csvfile, delimiter=',')            
                                        writer.writerow([str(j),str(k),str(vehicles[j].position[0]),str(vehicles[j].position[1]),str(vehicles[k].position[0]),str(vehicles[k].position[1]),str(i),"Collision"])
                        # Otherwise, if the second closest doesn't meet the MTL, the closest message still succeeds
                        # because we already determined that the closest meets the MTL
                        elif transmissions[1]["received_power"] < p.receiver_min_trigger_level:
                            #print("Second one")
                            for ell in range(len(transmissions) - 1):
                                if transmissions[ell + 1]["successful_decode"]:
                                    transmissions[ell + 1]["successful_decode"] = False
                                    false_from_collision += 1
                                    with open('Collisions.csv', 'a', newline='') as csvfile: #after first run change to a
                                        writer = csv.writer(csvfile, delimiter=',')            
                                        writer.writerow([str(j),str(k),str(vehicles[j].position[0]),str(vehicles[j].position[1]),str(vehicles[k].position[0]),str(vehicles[k].position[1]),str(i),"Collision"])
                        # The only other option is that the closest and second-closest meet the MTL, but the difference
                        # in power is not great enough for the closest to be selected, so all messages fail
                        else:
                            
                            for ell in range(len(transmissions)):  # make them all False
                                
                                if transmissions[ell]["successful_decode"]:
                                  
                                    transmissions[ell]["successful_decode"] = False
                                    false_from_collision += 1
                                    with open('Collisions.csv', 'a', newline='') as csvfile: #after first run change to a
                                        writer = csv.writer(csvfile, delimiter=',')            
                                        writer.writerow([str(j),str(k),str(vehicles[j].position[0]),str(vehicles[j].position[1]),str(vehicles[k].position[0]),str(vehicles[k].position[1]),str(i),"Collision"])
                    # If the closest transmission doesn't meet MTL, they all fail
                    else:
                        #print("Fourth one")
                        for ell in range(len(transmissions)):  # make them all False
                            if transmissions[ell]["successful_decode"]:
                                transmissions[ell]["successful_decode"] = False
                                false_from_collision += 1
                                with open('Collisions.csv', 'a', newline='') as csvfile: #after first run change to a
                                    writer = csv.writer(csvfile, delimiter=',')            
                                    writer.writerow([str(j),str(k),str(vehicles[j].position[0]),str(vehicles[j].position[1]),str(vehicles[k].position[0]),str(vehicles[k].position[1]),str(i),"Collision"])

        #vehicles[j].transmissions
        for k in range(p.num_vehicles):
            for j in range(len(vehicles[k].transmissions)):
                count_success += 1
        print("Successful decodes: ",count_success)
        num_MSO_collisions += len(collision_objects)  # record number of MSO collisions in this second
        overall_collision_objects.append(collision_objects)  # record collision objects for plotting
        collisions_per_second.append(len(collision_objects))  # record number of MSO collisions in this second
    #bar.finish()  # finish the progress bar when all done

    # This message helps identify the simulation that results are for
    #print("Simulation of {} vehicles for {} seconds with an ERP of {}".format(p.num_vehicles, p.num_seconds,
     #                                                                         p.effective_radiated_power))
    # This prints the number of __-way MSO collisions
    counts = []
    # for i in range(2, p.num_vehicles + 1):
    #     count = vehicles_per_collision.count(i)
    #     if count != 0:
    #         counts.append(count)
    #         print("Number of {}-way MSO collisions: {}".format(i, count))
    totalMso = (p.num_seconds*p.num_vehicles * (p.num_vehicles-1))
    with open('Jonathan_test_100_2.csv', 'a', newline='') as csvfile: #after first run change to a
        writer = csv.writer(csvfile, delimiter=',')            
        #writer.writerow(["Num Vehicles","Watts","Total MSO","Area Radius","overlap","chance","closeness","collisions","MTL","High Power","seed"])
        writer.writerow([str(p.num_vehicles),str(ERP_TYPE),str(totalMso),str(p.radius_of_area),
            str(false_from_message_overlap),str(false_by_chance),str(false_from_closeness),str(false_from_collision),str(false_from_minimum_trigger_level),str(false_from_high_power ),str(seed)])
    
    
    
    
    #print("Overlap: ", false_from_message_overlap)
    #print("Chance: ",false_by_chance)
    #print("Closeness: ",false_from_closeness)
    #print("Collision: ",false_from_collision)
    #print("MTL: ",false_from_minimum_trigger_level)
    #print("High Power: ",false_from_high_power)
    #print("Count Coulton: ", count_coulton)
    #print("Total MSO collisions:", num_MSO_collisions)
    # print("Mean collisions per second: {} \nVariance: {} \nStandard Deviation: {}".format(
    #     np.mean(collisions_per_second), np.var(collisions_per_second), np.std(collisions_per_second)))
    # unsuccessful_decodes = false_from_closeness + false_from_collision + false_from_message_overlap + \
    #                        false_from_minimum_trigger_level + false_from_high_power + false_by_chance
    # print("Unsuccessful decodes:", unsuccessful_decodes)
    # print("Unsuccessful decodes / Desired decodes: {}/{} = {} %".format(
    #     unsuccessful_decodes, p.num_desired_decodes, round(100 * unsuccessful_decodes / p.num_desired_decodes, 2)))
    # print("Failed from MTL: {}--{} %".format(
    #     false_from_minimum_trigger_level, round(100 * false_from_minimum_trigger_level / p.num_desired_decodes, 2)))

    # # Stats calculated in one way
    # print("Failed by chance: {}--{} %".format(
    #     false_by_chance, round(100 * false_by_chance / p.num_desired_decodes, 2)))
    # print("Failed from closeness: {}--{} %".format(
    #     false_from_closeness, round(100 * false_from_closeness / p.num_desired_decodes, 2)))
    # print("Failed from message overlap: {}--{} %".format(
    #     false_from_message_overlap, round(100 * false_from_message_overlap / p.num_desired_decodes, 2)))
    # print("Failed from MSO collision: {}--{} %".format(
    #     false_from_collision, round(100 * false_from_collision / p.num_desired_decodes, 2)))
    # print("Failed from too high power: {}--{} %".format(
    #     false_from_high_power, round(100 * false_from_high_power / p.num_desired_decodes, 2)))

    # # Stats calculated in a more meaningful way (see conference paper...)
    # actual_potential_decodes = p.num_desired_decodes - false_from_minimum_trigger_level
    # print("\nFailed by chance: {}--{} %".format(
    #     false_by_chance, round(100 * false_by_chance / actual_potential_decodes, 2)))
    # print("Failed from closeness: {}--{} %".format(
    #     false_from_closeness, round(100 * false_from_closeness / actual_potential_decodes, 2)))
    # print("Failed from message overlap: {}--{} %".format(
    #     false_from_message_overlap, round(100 * false_from_message_overlap / actual_potential_decodes, 2)))
    # print("Failed from MSO collision: {}--{} %".format(
    #     false_from_collision, round(100 * false_from_collision / actual_potential_decodes, 2)))
    # print("Failed from too high power: {}--{} %".format(
    #     false_from_high_power, round(100 * false_from_high_power / actual_potential_decodes, 2)))
    # really_unsuccessful_decodes = false_from_closeness + false_from_collision + false_from_message_overlap + \
    #                               false_by_chance
    # print("Unsuccessful decodes:", unsuccessful_decodes)
    # print("Unsuccessful decodes / Desired decodes: {}/{} = {} %".format(
    #     really_unsuccessful_decodes, actual_potential_decodes, round(100 * really_unsuccessful_decodes /
    #                                                                  actual_potential_decodes, 2)))

    # This figure plots vehicle positions as well as starting points and collisions. It adds a circle to the plot which
    # might be too specific. This plot was really designed around the "crossing the circle" case. There is figure in our
    # SciTech conference paper that was made using this code. I'm not really going to mess with this right now.
    
    # plt.rcParams.update({"font.size": 15})  # Adjust font size for plot
    # fig = plt.figure(1)  # Create figure
    # circle = plt.Circle((0, 0), p.radius_of_area, edgecolor="k", facecolor="none")  # Create black circle
    # ax = fig.add_subplot(111)  # Create the axis that we will add the scatters to. No idea what the 111 does, but keep
    # plt.gca().add_artist(circle)  # Add the previously created black circle to the plot
    # plt.title("2D UAS Positions")  # Title for the plot if collisions are not plotted
    # grays = ["Gray", "Silver", "Gainsboro", "DimGray", "LightGray"]
    # point_size = 10
    # plot_collisions = False
    # legend_not_added = True  # There is probably an easier way to do this
    # if p.num_vehicles < 30:
    #     plot_collisions = True
    # for i in range(p.num_vehicles):
    #     north_vals = []
    #     east_vals = []
    #     for j in range(1, p.num_seconds):
    #         north_vals.append(p.vehicle_positions[i][j][0])
    #         east_vals.append(p.vehicle_positions[i][j][1])
    #     gray = grays[i % len(grays)]
    #     ax.scatter(east_vals, north_vals, c=gray, s=point_size)
    #     if legend_not_added:
    #         legend_not_added = False
    #         ax.scatter(p.vehicle_positions[i][0][1], p.vehicle_positions[i][0][0],
    #                    marker="v", c="Black", s=point_size * 5, label="UAS Starting Position")
    #     else:
    #         ax.scatter(p.vehicle_positions[i][0][1], p.vehicle_positions[i][0][0],
    #                    marker="v", c="Black", s=point_size * 5)
    # if plot_collisions:
    #     collisions_2d = []
    #     legend_labels = []
    #     color_index = 0
    #     label_index = 0
    #     colors = ["Red", "DarkOrange", "Yellow", "Lime", "Aqua", "DodgerBlue", "Indigo", "BlueViolet", "Fuchsia"]
    #     for i in range(num_MSO_collisions):
    #         legend_labels.append(legend_labels.append("Collision {}".format(i + 1)))
    #     for i in range(p.num_seconds):
    #         for j in range(len(overall_collision_objects[i])):
    #             color = colors[color_index]
    #             if len(overall_collision_objects[i][j]["vehicles_involved"]) > 1:
    #                 color_index += 1
    #                 if color_index == 8:
    #                     color_index = 0
    #             for k in range(len(overall_collision_objects[i][j]["vehicles_involved"])):
    #                 vehicle_id = overall_collision_objects[i][j]["vehicles_involved"][k].vehicle_identifier
    #                 if k == 0:
    #                     label_index += 1
    #                     collisions_2d.append(ax.scatter(p.vehicle_positions[vehicle_id][i][1],
    #                                                     p.vehicle_positions[vehicle_id][i][0],
    #                                                     marker="D", c=color, s=point_size * 2,
    #                                                     label="Collision {}".format(label_index)))
    #                 else:
    #                     ax.scatter(p.vehicle_positions[vehicle_id][i][1],
    #                                p.vehicle_positions[vehicle_id][i][0],
    #                                marker="D", c=color, s=point_size * 2)
    #     plt.legend(fontsize=8, bbox_to_anchor=(1, 0.7))
    #     plt.title("2D UAS Positions With MSO Collisions")
    # ax.set_aspect("equal")
    # ax.set_xlabel("X (m)")
    # ax.set_ylabel("Y (m)")
    # #plt.xlim([-10,110])
    # #plt.ylim([-10,10])
    # plt.show()
    
    
    end = time.time()
    print("Total runtime:", end - start)
