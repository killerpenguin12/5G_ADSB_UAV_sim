# Feb 1, 2021
# This is the main model simulation file.

import time
import numpy as np
import propagation_param as p
import random
from Vehicle import Vehicle
from progress.bar import IncrementalBar

MAX_DISTANCE = 0
POWER_BELOW_MTL = -94.0
if p.effective_radiated_power == 0.01:
    ERP_TYPE = "hundredth_watt"
    MAX_DISTANCE = 3446
    from hundredth_watt_dict import hundredth_watt_dict as power_dict
elif p.effective_radiated_power == 0.05:
    ERP_TYPE = "twentieth_watt"
    MAX_DISTANCE = 7705
    from twentieth_watt_dict import twentieth_watt_dict as power_dict
elif p.effective_radiated_power == 0.1:
    ERP_TYPE = "tenth_watt"
    MAX_DISTANCE = 10897
    from tenth_watt_dict import tenth_watt_dict as power_dict
elif p.effective_radiated_power == 1.0:
    ERP_TYPE = "one_watt"
    MAX_DISTANCE = 34457
    from one_watt_dict import one_watt_dict as power_dict
elif p.effective_radiated_power == 10.0:
    ERP_TYPE = "ten_watt"
    MAX_DISTANCE = 74081
    from ten_watt_dict import ten_watt_dict as power_dict
elif p.effective_radiated_power == 20.0:
    ERP_TYPE = "twenty_watt"
    MAX_DISTANCE = 74081
    from twenty_watt_dict import twenty_watt_dict as power_dict
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


def init_transmissions(vehicles, false_from_high_power, false_from_minimum_trigger_level, false_by_chance, j,
                       distances):
    vehicle_transmissions = []
    for k in range(p.num_vehicles):
        vehicle_transmissions.append(0)
    for k in range(vehicles[j].vehicle_identifier + 1, vehicles[j].vehicle_identifier + p.num_vehicles):
        distance = get_distance(vehicles, j, k)
        if distance == 0:
            distance = 1
        if distance < MAX_DISTANCE:
            received_power = power_dict.get(distance)
        else:
            received_power = POWER_BELOW_MTL
        meets_minimum_trigger_level = True
        if True:  # This is to test power being too high, and meeting the MTL, which may be an unreliable way to
            if received_power >= -90.0:
                if random.random() < 0.99:
                    meets_minimum_trigger_level = True
                else:
                    meets_minimum_trigger_level = False
                    false_by_chance += 1
            elif p.receiver_min_trigger_level <= received_power < -90.0:
                if random.random() < 0.9:
                    meets_minimum_trigger_level = True
                else:
                    meets_minimum_trigger_level = False
                    false_by_chance += 1
            else:
                meets_minimum_trigger_level = False
                false_from_minimum_trigger_level += 1
        transmission = {"receiver_ID": vehicles[k % p.num_vehicles].vehicle_identifier,
                        "distance": distance,
                        "received_power": received_power,
                        "successful_decode": meets_minimum_trigger_level}
        vehicle_transmissions[transmission["receiver_ID"]] = transmission
    vehicles[j].transmissions = vehicle_transmissions.copy()
    return false_from_high_power, false_from_minimum_trigger_level, false_by_chance


def evaluate_switching_modes(vehicles):
    false_from_closeness = 0
    for j in range(p.num_vehicles):
        for k in range(p.num_vehicles):  # this counts failing because you're switching modes.
            # It also counts failure to decode because you're on the same MSO
            if vehicles[j].transmissions[k] == 0:
                continue
            if (vehicles[j].message_start_opportunity - 9) <= \
                    vehicles[k].message_start_opportunity <= \
                    (vehicles[j].message_start_opportunity + 8):
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
                        collisions_in_time, filtered_mso_collisions, collision_distances):
    false_from_collision = 0
    for j in range(752, 3952):  # for each MSO
        count = seconds[i].count(j)  # count how many times that MSO occurred in the second
        if count > 1:  # if the MSO occurred more than once, there was an MSO collision
            # collisions_in_time.append(i)
            # do something to figure out which vehicles
            vehicles_involved = []  # records which vehicles are involved in the collision
            vehicle_ids_involved = []
            for k in range(p.num_vehicles):  # for each vehicle
                if vehicles[k].message_start_opportunity == j:  # if the MSO matches the one of the collision
                    vehicle_ids_involved.append(k)
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
        # filtered_mso_added = False
        for k in range(p.num_vehicles):  # for every vehicle...
            mso_to_check = vehicles[k].message_start_opportunity
            if (mso_to_check < (mso - 9)) or \
                    (mso_to_check > (
                            mso + 8)):  # that is not part of the collision (and not switching modes)
                transmissions = []
                # compare to each vehicle that was part of the collision
                for l in range(len(collision_objects[j]["vehicles_involved"])):
                    transmissions.append(collision_objects[j]["vehicles_involved"][l].transmissions[k])
                transmissions.sort(key=lambda transmission: transmission["distance"])
                if transmissions[0]["received_power"] >= p.receiver_min_trigger_level:
                    if transmissions[0]["received_power"] - transmissions[1]["received_power"] >= \
                            3000:
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
    num_MSO_collisions = 0
    vehicles = []  # Declare list of vehicles, empty for now
    msos = []  # Will store every MSO calculated, so we can look at the frequency of each MSO
    seconds = []  # List that will have a list of MSOs for every second, to organize them better
    mso_collisions = []  # Will record any MSO that is calculated by two or more vehicles in the same second
    filtered_mso_collisions = []
    vehicles_per_collision = []  # Will keep track of how many __-way MSO collisions there are
    collisions_in_time = []
    collision_distances = []
    collisions_per_second = []
    distances = []
    overall_collision_objects = []
    false_from_closeness = 0
    false_from_collision = 0
    false_from_message_overlap = 0
    false_from_minimum_trigger_level = 0
    false_by_chance = 0
    false_from_high_power = 0
    bar = IncrementalBar("Simulating...", max=p.num_seconds)
    # Initialize vehicles
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
                vehicles[j].pseudo_random_number(0)  # Calculate initial number used later
        # Begin MSO calculations
        for j in range(p.num_vehicles):
            mso = vehicles[j].full_mso_range()  # calculate MSO
            false_from_high_power, false_from_minimum_trigger_level, false_by_chance = \
                init_transmissions(vehicles, false_from_high_power,
                                   false_from_minimum_trigger_level, false_by_chance, j, distances)
            vehicles[j].frame += 1  # increase frame
            msos.append(mso)  # record MSO
            seconds[i].append(mso)  # record MSO in a way that helps with collisions
        false_from_closeness += evaluate_switching_modes(vehicles)  # Evaluate messages lost from transmitting/receiving transition
        false_from_message_overlap += evaluate_consecutive_msos(vehicles)  # Evaluate messages lost from partial overlap
        false_from_collision += evaluate_collisions(seconds, vehicles, i, collision_objects, vehicles_per_collision,
                                                    mso_collisions, collisions_in_time, filtered_mso_collisions,
                                                    collision_distances) # Evaluate MSO collisions
        num_MSO_collisions += len(collision_objects)
        overall_collision_objects.append(collision_objects)
        collisions_per_second.append(len(collision_objects))
        bar.next()
    bar.finish()

    print("Simulation of {} vehicles for {} seconds with an ERP of {}".format(p.num_vehicles, p.num_seconds,
                                                                              p.effective_radiated_power))
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
                           false_from_minimum_trigger_level + false_from_high_power + false_by_chance
    print("Unsuccessful decodes:", unsuccessful_decodes)
    print("Unsuccessful decodes / Desired decodes: {}/{} = {} %".format(
        unsuccessful_decodes, p.num_desired_decodes, round(100 * unsuccessful_decodes / p.num_desired_decodes, 2)))
    print("Failed from MTL: {}--{} %".format(
        false_from_minimum_trigger_level, round(100 * false_from_minimum_trigger_level / p.num_desired_decodes, 2)))
    print("Failed by chance: {}--{} %".format(
        false_by_chance, round(100 * false_by_chance / p.num_desired_decodes, 2)))
    print("Failed from closeness: {}--{} %".format(
        false_from_closeness, round(100 * false_from_closeness / p.num_desired_decodes, 2)))
    print("Failed from message overlap: {}--{} %".format(
        false_from_message_overlap, round(100 * false_from_message_overlap / p.num_desired_decodes, 2)))
    print("Failed from MSO collision: {}--{} %".format(
        false_from_collision, round(100 * false_from_collision / p.num_desired_decodes, 2)))
    print("Failed from too high power: {}--{} %".format(
        false_from_high_power, round(100 * false_from_high_power / p.num_desired_decodes, 2)))

    actual_potential_decodes = p.num_desired_decodes - false_from_minimum_trigger_level
    print("\nFailed by chance: {}--{} %".format(
        false_by_chance, round(100 * false_by_chance / actual_potential_decodes, 2)))
    print("Failed from closeness: {}--{} %".format(
        false_from_closeness, round(100 * false_from_closeness / actual_potential_decodes, 2)))
    print("Failed from message overlap: {}--{} %".format(
        false_from_message_overlap, round(100 * false_from_message_overlap / actual_potential_decodes, 2)))
    print("Failed from MSO collision: {}--{} %".format(
        false_from_collision, round(100 * false_from_collision / actual_potential_decodes, 2)))
    print("Failed from too high power: {}--{} %".format(
        false_from_high_power, round(100 * false_from_high_power / actual_potential_decodes, 2)))
    really_unsuccessful_decodes = false_from_closeness + false_from_collision + false_from_message_overlap + \
                                  false_by_chance
    print("Unsuccessful decodes:", unsuccessful_decodes)
    print("Unsuccessful decodes / Desired decodes: {}/{} = {} %".format(
        really_unsuccessful_decodes, actual_potential_decodes, round(100 * really_unsuccessful_decodes /
                                                                     actual_potential_decodes, 2)))


if __name__ == "__main__":
    start = time.time()
    main()
    end = time.time()
    print("Total runtime:", end - start)
