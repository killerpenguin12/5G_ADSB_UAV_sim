# Feb 1, 2021
# I believe this is the main sim file, but with random MSOs instead of MSOs determined by positional data.
# It yielded basically the same results as the main sim, so I haven't done much with it since

import csv
import time
import numpy as np
import propagation_param as p
import random
from Vehicle import Vehicle
import matplotlib.pyplot as plt
import mplcursors
from progress.bar import IncrementalBar

MAX_DISTANCE = 0
POWER_BELOW_MTL = -94.0
if p.effective_radiated_power == 0.01:
    MAX_DISTANCE = 3446
    from hundredth_watt_dict import hundredth_watt_dict as power_dict
elif p.effective_radiated_power == 0.05:
    MAX_DISTANCE = 7705
    from twentieth_watt_dict import twentieth_watt_dict as power_dict
elif p.effective_radiated_power == 0.1:
    MAX_DISTANCE = 10897
    from tenth_watt_dict import tenth_watt_dict as power_dict
elif p.effective_radiated_power == 1.0:
    MAX_DISTANCE = 34457
    from one_watt_dict import one_watt_dict as power_dict
"""
To run the code:
- Install all of the necessary packages
- Verify the correct folder_name num_vehicles, and num_vehicles in propagation_param.py
- Verify the correct conditions in the if statements in the param file
"""


# Populate vehicles list with vehicles
def init_vehicles(vehicles):
    for i in range(p.num_vehicles):
        uas = Vehicle((0, 0, 0), random.randrange(3601), i)
        vehicles.append(uas)


def get_distance(vehicles, j, k):
    return np.int(np.ceil(np.sqrt((vehicles[j].position[0] - vehicles[k % p.num_vehicles].position[0]) ** 2.0 +
                                  (vehicles[j].position[1] - vehicles[k % p.num_vehicles].position[1]) ** 2.0 +
                                  (vehicles[j].position[2] - vehicles[k % p.num_vehicles].position[2]) ** 2.0)))


def init_transmissions(vehicles, false_from_high_power, false_from_minimum_trigger_level, j):
    vehicle_transmissions = []
    for k in range(p.num_vehicles):
        vehicle_transmissions.append(0)
    for k in range(vehicles[j].vehicle_identifier + 1, vehicles[j].vehicle_identifier + p.num_vehicles):
        distance = get_distance(vehicles, j, k)
        if distance < MAX_DISTANCE:
            received_power = power_dict.get(distance)
        else:
            received_power = POWER_BELOW_MTL
        meets_minimum_trigger_level = True
        if True:  # This is to test power being too high, and meeting the MTL, which may be an unreliable way to
            # accumulate unsuccessful decodes
            if received_power > -3.0:
                meets_minimum_trigger_level = False
                false_from_high_power += 1
            elif received_power >= p.receiver_min_trigger_level:
                meets_minimum_trigger_level = True
            else:
                meets_minimum_trigger_level = False
                false_from_minimum_trigger_level += 1
        transmission = {"receiver_ID": vehicles[k % p.num_vehicles].vehicle_identifier,
                        "distance": distance,
                        "received_power": received_power,
                        "successful_decode": meets_minimum_trigger_level}
        vehicle_transmissions[transmission["receiver_ID"]] = transmission
    vehicles[j].transmissions = vehicle_transmissions.copy()
    return false_from_high_power, false_from_minimum_trigger_level


def evaluate_switching_modes(vehicles):
    false_from_closeness = 0
    for j in range(p.num_vehicles):
        for k in range(p.num_vehicles):  # this counts failing because you're switching modes.
            # It also counts failure to decode because you're on the same MSO
            if vehicles[j].transmissions[k] == 0:
                continue
            if (vehicles[j].message_start_opportunity >=
                (vehicles[vehicles[j].transmissions[k]["receiver_ID"]].message_start_opportunity - 8)) and \
                    (vehicles[j].message_start_opportunity <=
                     (vehicles[vehicles[j].transmissions[k]["receiver_ID"]].message_start_opportunity + 8)):
                if vehicles[j].transmissions[k]["successful_decode"]:
                    vehicles[j].transmissions[k]["successful_decode"] = False
                    false_from_closeness += 1
    return false_from_closeness


def evaluate_consecutive_msos(vehicles):
    false_from_message_overlap = 0
    sorted_msos = vehicles.copy()
    sorted_msos.sort(key=lambda vehicle: vehicle.message_start_opportunity)
    last_mso = sorted_msos[0].message_start_opportunity
    num_consecutive = 0
    for j in range(1, len(sorted_msos)):
        if sorted_msos[j].message_start_opportunity == last_mso + 1 or \
                sorted_msos[j].message_start_opportunity == last_mso:
            if sorted_msos[j].message_start_opportunity == last_mso + 1:
                num_consecutive += 1
            if num_consecutive % 2:
                for k in range(p.num_vehicles):
                    if sorted_msos[j].transmissions[k] != 0:
                        if sorted_msos[j].transmissions[k]["successful_decode"]:
                            sorted_msos[j].transmissions[k]["successful_decode"] = False
                            false_from_message_overlap += 1
        else:
            num_consecutive = 0
        last_mso = sorted_msos[j].message_start_opportunity
    return false_from_message_overlap


def evaluate_collisions(seconds, vehicles, i, collision_objects, vehicles_per_collision, mso_collisions,
                        collisions_in_time, filtered_mso_collisions):
    false_from_collision = 0
    for j in range(752, 3952):  # for each MSO
        count = seconds[i].count(j)  # count how many times that MSO occurred in the second
        if count > 1:  # if the MSO occurred more than once, there was an MSO collision
            collisions_in_time.append(i)
            # do something to figure out which vehicles
            vehicles_involved = []  # records which vehicles are involved in the collision
            for k in range(p.num_vehicles):  # for each vehicle
                if vehicles[k].message_start_opportunity == j:  # if the MSO matches the one of the collision
                    vehicles_involved.append(
                        vehicles[k])  # record the vehicle number, which corresponds to an index in the vehicles list
            vehicles_per_collision.append(count)  # Record how many vehicles per collision
            mso_collisions.append(j)  # record MSO where collision happened
            mso_collision = {"second": i, "message_start_opportunity": j,
                             "vehicles_involved": vehicles_involved}  # record the collision as object
            collision_objects.append(mso_collision)  # add the new collision to the list
    # have this loop check in the case of collisions
    for j in range(len(collision_objects)):  # for every collision
        mso = collision_objects[j]["message_start_opportunity"]
        filtered_mso_added = False
        for k in range(p.num_vehicles):  # for every vehicle...
            mso_to_check = vehicles[k].message_start_opportunity
            if (mso_to_check < (mso - 8)) or \
                    (mso_to_check > (mso + 8)):  # that is not part of the collision (and not switching modes)
                transmissions = []
                # compare to each vehicle that was part of the collision
                for l in range(len(collision_objects[j]["vehicles_involved"])):
                    transmissions.append(collision_objects[j]["vehicles_involved"][l].transmissions[k])
                transmissions.sort(key=lambda transmission: transmission["distance"])
                if transmissions[0]["received_power"] >= p.receiver_min_trigger_level:
                    if not filtered_mso_added:
                        filtered_mso_collisions.append(mso)
                        filtered_mso_added = True
                    if transmissions[0]["received_power"] - transmissions[1]["received_power"] >= \
                            p.power_difference_decibel:
                        for l in range(len(transmissions) - 1):
                            if transmissions[l + 1]["successful_decode"]:
                                transmissions[l + 1]["successful_decode"] = False
                                false_from_collision += 1
                    elif transmissions[1]["received_power"] < p.receiver_min_trigger_level:
                        for l in range(len(transmissions) - 1):
                            if transmissions[l + 1]["successful_decode"]:
                                transmissions[l + 1]["successful_decode"] = False
                                false_from_collision += 1
                    else:
                        for l in range(len(transmissions)):  # make them all False
                            if transmissions[l]["successful_decode"]:
                                transmissions[l]["successful_decode"] = False
                                false_from_collision += 1
                else:
                    for l in range(len(transmissions)):  # make them all False
                        if transmissions[l]["successful_decode"]:
                            transmissions[l]["successful_decode"] = False
                            false_from_collision += 1
    return false_from_collision


def main():
    vehicles = []  # Declare list of vehicles, empty for now
    msos = []  # Will store every MSO calculated, so we can look at the frequency of each MSO
    seconds = []  # List that will have a list of MSOs for every second, to organize them better
    mso_collisions = []  # Will record any MSO that is calculated by two or more vehicles in the same second
    filtered_mso_collisions = []
    vehicles_per_collision = []  # Will keep track of how many __-way MSO collisions there are
    collisions_in_time = []
    collisions_per_second = []
    false_from_closeness = 0
    false_from_collision = 0
    false_from_message_overlap = 0
    false_from_minimum_trigger_level = 0
    false_from_high_power = 0
    bar = IncrementalBar("Simulating...", max=p.num_seconds)
    init_vehicles(vehicles)
    # The rest of the time we do the other MSO calculation
    for i in range(p.num_seconds):
        # Puts a list of MSOs in for every second in seconds array
        msos_per_second = []
        seconds.append(msos_per_second)
        collision_objects = []
        # Before calculating MSOs, update the vehicle positions
        for j in range(p.num_vehicles):
            vehicles[j].update_position(p.vehicle_positions[j][i])
            if i == 0:
                vehicles[j].pseudo_random_number(0)
        # Begin MSO calculations
        for j in range(p.num_vehicles):
            mso = random.randint(752, 3952)
            vehicles[j].message_start_opportunity = mso
            false_from_high_power, false_from_minimum_trigger_level = \
                init_transmissions(vehicles, false_from_high_power, false_from_minimum_trigger_level, j)
            vehicles[j].frame += 1  # increase frame
            msos.append(mso)  # record MSO
            seconds[i].append(mso)  # record MSO in a way that helps with collisions
        false_from_closeness += evaluate_switching_modes(vehicles)
        false_from_message_overlap += evaluate_consecutive_msos(vehicles)
        false_from_collision += evaluate_collisions(seconds, vehicles, i, collision_objects, vehicles_per_collision,
                                                    mso_collisions, collisions_in_time, filtered_mso_collisions)
        collisions_per_second.append(len(collision_objects))
        bar.next()
    bar.finish()

    # This prints the number of __-way MSO collisions
    vehicles_per_collision_bins = 0
    counts = []
    for i in range(2, p.num_vehicles + 1):
        count = vehicles_per_collision.count(i)
        if count != 0:
            counts.append(count)
            vehicles_per_collision_bins += 1
            print("Number of {}-way MSO collisions: {}".format(i, count))

    x = np.arange(2, vehicles_per_collision_bins + 2, step=1)
    print("Total MSO collisions:", len(mso_collisions))
    print("Mean collisions per second: {} \nVariance: {} \nStandard Deviation: {}".format(
        np.mean(collisions_per_second), np.var(collisions_per_second), np.std(collisions_per_second)))
    unsuccessful_decodes = false_from_closeness + false_from_collision + false_from_message_overlap + \
        false_from_minimum_trigger_level + false_from_high_power
    print("Unsuccessful decodes:", unsuccessful_decodes)
    print("Unsuccessful decodes / Desired decodes: {}/{} = {} %".format(
        unsuccessful_decodes, p.num_desired_decodes, round(100 * unsuccessful_decodes / p.num_desired_decodes, 2)))
    print("Failed from MTL: {}--{} %".format(
        false_from_minimum_trigger_level, round(100 * false_from_minimum_trigger_level / p.num_desired_decodes, 2)))
    print("Failed from closeness: {}--{} %".format(
        false_from_closeness, round(100 * false_from_closeness / p.num_desired_decodes, 2)))
    print("Failed from message overlap: {}--{} %".format(
        false_from_message_overlap, round(100 * false_from_message_overlap / p.num_desired_decodes, 2)))
    print("Failed from MSO collision: {}--{} %".format(
        false_from_collision, round(100 * false_from_collision / p.num_desired_decodes, 2)))
    print("Failed from too high power: {}--{} %".format(
        false_from_high_power, round(100 * false_from_high_power / p.num_desired_decodes, 2)))

    mso_collisions.sort()
    filtered_mso_collisions.sort()
    if mso_collisions == filtered_mso_collisions:
        print("they are the same")

    plot_type = "png"
    plt.figure(1)
    plt.hist(msos, bins=3200)
    mplcursors.cursor()
    plt.title("Occurrences of Specific MSOs")
    plt.xlabel("MSO")
    plt.ylabel("Occurrences of MSO")
    plt.savefig("fig1." + plot_type)

    plt.figure(2)
    plt.hist(mso_collisions, bins=3200)
    mplcursors.cursor()
    plt.title("Occurrences of MSO Collisions on a Specific MSO")
    plt.xlabel("MSO")
    plt.ylabel("Occurrences of Collisions on MSO")
    plt.savefig("fig2." + plot_type)

    plt.figure(3)
    plt.bar(x, counts)
    plt.xticks(x)
    mplcursors.cursor()
    plt.title("MSO Collisions by Number of Vehicles Involved")
    plt.xlabel("Vehicles per Collision")
    plt.ylabel("Number of Collisions")
    plt.savefig("fig3." + plot_type)

    plt.figure(4)
    plt.hist(collisions_in_time, bins=p.num_seconds)
    mplcursors.cursor()
    plt.title("Occurrences of Collisions in Time")
    plt.xlabel("Second")
    plt.ylabel("Number of Collisions")
    plt.savefig("fig4." + plot_type)

    plt.rcParams.update({"font.size": 15})
    fig = plt.figure(5)
    circle = plt.Circle((0, 0), 29632, edgecolor="k", facecolor="none")
    ax = fig.add_subplot(111)
    plt.gca().add_artist(circle)

    for i in range(p.num_vehicles):
        north_vals = []
        east_vals = []
        for j in range(p.num_seconds):
            north_vals.append(p.vehicle_positions[i][j][0])
            east_vals.append(p.vehicle_positions[i][j][1])
        ax.scatter(north_vals, east_vals, s=1)

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    plt.savefig("fig5." + plot_type)

    plt.figure(6)
    plt.hist(filtered_mso_collisions, bins=3200)
    mplcursors.cursor()
    plt.title("Occurrences of MSO Collisions--Filtered")
    plt.xlabel("MSO")
    plt.ylabel("Occurrences of Collisions on MSO")
    plt.savefig("fig6." + plot_type)

    plt.show()
    with open("collisions_per_sec{}V{}S".format(p.num_vehicles, p.num_seconds), "w", newline="") as myfile:
        wr = csv.writer(myfile)
        wr.writerow(collisions_per_second)


if __name__ == "__main__":
    start = time.time()
    main()
    end = time.time()
    print("Total runtime:", end - start)


# You need a simulation version where MSOs are randomly assigned rather than assigned by the algorithm
