'''
Most current main file. 5/11/2020
'''
import time
import numpy as np
import propagation_param as p
import random
from Vehicle import vehicle
import propagationFunctions as pf
import matplotlib.pyplot as plt
import mplcursors
from myStructs import MSO_collision, transmission
from progress.bar import IncrementalBar
# import cProfile
'''
To run the code:
- Install all of the necessary packages
- Verify the correct folderName and num_vehicles in
  propagation_param.py
- Verify the correct conditions to the if statements in the param file
'''


# Populate vehicles list with vehicles
def initVehicles(vehicles):
    for i in range(p.num_vehicles):
        uas = vehicle((0, 0, 0), random.randrange(3601), i)  # FIXME indexing
        vehicles.append(uas)


def initTransmissions(vehicles, false_from_high_power, false_from_MTL, j):
    myTransmissions = []
    for k in range(p.num_vehicles):
        myTransmissions.append(0)
    for k in range(vehicles[j].vehicle_identifier + 1, vehicles[j].vehicle_identifier + p.num_vehicles):
        power_dBm = pf.combinedReceivedPower(vehicles[j], vehicles[k % p.num_vehicles])
        meets_MTL = True
        if False:  # This is to test power being too high, and meeting the MTL, which may be an unreliable way to
            # accumulate unsuccessful decodes
            if power_dBm > -3.0:
                meets_MTL = False
                false_from_high_power += 1
            elif power_dBm >= p.receiver_min_trigger_level:
                meets_MTL = True
            else:
                meets_MTL = False
                false_from_MTL += 1
        newTransmission = transmission(vehicles[k % p.num_vehicles].vehicle_identifier,
                                       pf.distanceBetweenVehicles(vehicles[j], vehicles[k % p.num_vehicles]),
                                       power_dBm, meets_MTL)
        myTransmissions[newTransmission.receiver_ID] = newTransmission
    vehicles[j].transmissions = myTransmissions.copy()
    return false_from_high_power, false_from_MTL

def evaluateSwitchingModes(vehicles):
    false_from_closeness = 0
    for j in range(p.num_vehicles):
        for k in range(p.num_vehicles):  # this counts failing because you're switching modes.
                                         # It also counts failure to decode because you're on the same MSO
            if vehicles[j].transmissions[k] == 0:
                continue
            if ((vehicles[j].MSO >= (vehicles[vehicles[j].transmissions[k].receiver_ID].MSO - 8)) and (
                    vehicles[j].MSO <= (vehicles[vehicles[j].transmissions[k].receiver_ID].MSO + 8))):
                if vehicles[j].transmissions[k].successful_decode:
                    vehicles[j].transmissions[k].successful_decode = False
                    false_from_closeness += 1
    return false_from_closeness

def evaluateConsecutiveMSOs(vehicles):
    false_from_message_overlap = 0
    sorted_MSOs = vehicles.copy()
    sorted_MSOs.sort(key=lambda vehicle: vehicle.MSO)
    last_MSO = sorted_MSOs[0].MSO
    num_consecutive = 0
    for j in range(1, len(sorted_MSOs)):
        if sorted_MSOs[j].MSO == last_MSO + 1 or sorted_MSOs[j].MSO == last_MSO:
            if sorted_MSOs[j].MSO == last_MSO + 1:
                num_consecutive += 1
            if num_consecutive % 2:
                for k in range(p.num_vehicles):
                    if sorted_MSOs[j].transmissions[k] != 0:
                        if sorted_MSOs[j].transmissions[k].successful_decode:
                            sorted_MSOs[j].transmissions[k].successful_decode = False
                            false_from_message_overlap += 1
        else:
            num_consecutive = 0
        last_MSO = sorted_MSOs[j].MSO
    return false_from_message_overlap

def evaluateCollisions(seconds, vehicles, i, collision_objects, vehicles_per_collision, MSO_collisions):
    false_from_collision = 0
    for j in range(752, 3952):  # for each MSO
        count = seconds[i].count(j)  # count how many times that MSO occurred in the second
        if count > 1:  # if the MSO occurred more than once, there was an MSO collision
            # do something to figure out which vehicles
            vehicles_involved = []  # records which vehicles are involved in the collision
            for k in range(p.num_vehicles):  # for each vehicle
                if vehicles[k].MSO == j:  # if the MSO matches the one of the collision
                    vehicles_involved.append(
                        vehicles[k])  # record the vehicle number, which corresponds to an index in the vehicles list
            vehicles_per_collision.append(count)  # Record how many vehicles per collision
            MSO_collisions.append(j)  # record MSO where collision happened
            new_collision = MSO_collision(i, j, vehicles_involved)  # record the collision as object
            collision_objects.append(new_collision)  # add the new collision to the list
    # have this loop check in the case of collisions
    for j in range(len(collision_objects)):  # for every collision
        MSO = collision_objects[j].MSO
        for k in range(p.num_vehicles):  # for every vehicle...
            MSO_to_check = vehicles[k].individual_MSOs[i]
            if (MSO_to_check < (MSO - 8)) or (
                    MSO_to_check > (MSO + 8)):  # that is not part of the collision (and not switching modes)
                transmissions = []
                for l in range(len(collision_objects[j].vehicles_involved)):  # compare to each vehicle that was part of the collision
                    transmissions.append(collision_objects[j].vehicles_involved[l].transmissions[k])
                transmissions.sort(key=lambda transmission: transmission.distance)
                if transmissions[0].received_power >= p.receiver_min_trigger_level:
                    if transmissions[0].received_power - transmissions[1].received_power >= p.power_difference_dB:
                        for l in range(len(transmissions) - 1):
                            if transmissions[l + 1].successful_decode:
                                transmissions[l + 1].successful_decode = False
                                false_from_collision += 1
                    else:
                        for l in range(len(transmissions)):  # make them all False
                            if transmissions[l].successful_decode:
                                transmissions[l].successful_decode = False
                                false_from_collision += 1
                else:
                    for l in range(len(transmissions)):  # make them all False
                        if transmissions[l].successful_decode:
                            transmissions[l].successful_decode = False
                            false_from_collision += 1
    return false_from_collision

def main():
    vehicles = []  # Declare list of vehicles, empty for now
    MSOs = []  # Will store every MSO calculated, so we can look at the frequency of each MSO
    seconds = []  # List that will have a list of MSOs for every second, to organize them better
    MSO_collisions = []  # Will record any MSO that is calculated by two or more vehicles in the same second
    vehicles_per_collision = []  # Will keep track of how many __-way MSO collisions there are
    false_from_closeness = 0
    false_from_collision = 0
    false_from_message_overlap = 0
    false_from_MTL = 0
    false_from_high_power = 0
    bar = IncrementalBar('Simulating...', max=p.num_seconds)
    initVehicles(vehicles)
    # The rest of the time we do the other MSO calculation
    for i in range(p.num_seconds):
        # Puts a list of MSOs in for every second in seconds array
        MSOs_per_second = []
        seconds.append(MSOs_per_second)
        collision_objects = []
        # Before calculating MSOs, update the vehicle positions
        for j in range(p.num_vehicles):
            vehicles[j].updatePosition(p.vehiclePositions[j][i])
            if i == 0:
                vehicles[j].pseudoRandomNumberR(0)
        # Begin MSO calculations
        for j in range(p.num_vehicles):
            MSO = vehicles[j].fullMSORange()  # calculate MSO
            false_from_high_power, false_from_MTL = initTransmissions(vehicles, false_from_high_power, false_from_MTL, j)
            vehicles[j].m += 1  # increase frame
            MSOs.append(MSO)  # record MSO
            seconds[i].append(MSO)  # record MSO in a way that helps with collisions
        false_from_closeness += evaluateSwitchingModes(vehicles)
        false_from_message_overlap += evaluateConsecutiveMSOs(vehicles)
        false_from_collision += evaluateCollisions(seconds, vehicles, i, collision_objects, vehicles_per_collision, MSO_collisions)
        bar.next()
    bar.finish()

    # This prints the number of __-way MSO collisions
    vehicles_per_collision_bins = 0
    counts = []
    for i in range(2, p.num_vehicles + 1):
        count = 0
        count = vehicles_per_collision.count(i)
        if (count != 0):
            counts.append(count)
            vehicles_per_collision_bins += 1
            print("Number of {}-way MSO collisions: {}".format(i, count))

    x = np.arange(2, vehicles_per_collision_bins + 2, step=1)
    print("Total MSO collisions:", len(MSO_collisions))
    unsuccessful_decodes = false_from_closeness + false_from_collision + false_from_message_overlap + false_from_MTL + false_from_high_power
    print("Unsuccessful decodes:", unsuccessful_decodes)
    print("Unsuccessful decodes / Desired decodes: {}/{} = {} %".format(unsuccessful_decodes, p.num_desired_decodes,
                                                                        round(
                                                                            100 * unsuccessful_decodes / p.num_desired_decodes,
                                                                            2)))
    print(
        "Failed from MTL: {}--{} %".format(false_from_MTL, round(100 * false_from_MTL / p.num_desired_decodes, 2)))
    print("Failed from closeness: {}--{} %".format(false_from_closeness,
                                                   round(100 * false_from_closeness / p.num_desired_decodes, 2)))
    print("Failed from message overlap: {}--{} %".format(false_from_message_overlap, round(
        100 * false_from_message_overlap / p.num_desired_decodes, 2)))
    print("Failed from MSO collision: {}--{} %".format(false_from_collision,
                                                       round(100 * false_from_collision / p.num_desired_decodes,
                                                             2)))
    print("Failed from too high power: {}--{} %".format(false_from_high_power,
                                                        round(100 * false_from_high_power / p.num_desired_decodes,
                                                              2)))

    plt.figure(1)
    plt.hist(MSOs, bins=3200)
    mplcursors.cursor()
    plt.title('Occurrences of Specific MSOs')
    plt.xlabel('MSO')
    plt.ylabel('Occurrences of MSO')

    plt.figure(2)
    plt.hist(MSO_collisions, bins=3200)
    mplcursors.cursor()
    plt.title('Occurrences of MSO Collisions on a Specific MSO')
    plt.xlabel('MSO')
    plt.ylabel('Occurrences of Collisions on MSO')

    plt.figure(3)
    plt.bar(x, counts)
    plt.xticks(x)
    mplcursors.cursor()
    plt.title('MSO Collisions by Number of Vehicles Involved')
    plt.xlabel('Vehicles per Collision')
    plt.ylabel('Number of Collisions')

    plt.rcParams.update({'font.size': 15})
    fig = plt.figure(4)
    # plt.gcf().subplots_adjust(left=0.15)
    circle = plt.Circle((0, 0), 29632, edgecolor='k', facecolor='none')
    ax = fig.add_subplot(111)
    plt.gca().add_artist(circle)

    for i in range(p.num_vehicles):
        n_vals = []
        e_vals = []
        for j in range(p.num_seconds):
            n_vals.append(p.vehiclePositions[i][j][0])
            e_vals.append(p.vehiclePositions[i][j][1])
        ax.scatter(n_vals, e_vals, s=1)

    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    # plt.show()

if __name__ == "__main__":
    start = time.time()
    main()
    end = time.time()
    print("Total runtime:", end - start)

    # So maybe I can put a bunch of distances and received powers in dictionaries. For distance, it can rely on two coordinates