# Feb 1 2021
# This header will eventually match another. I paste code here when I am
# swapping what file it is in? It's like a temp variable basically

import csv
import time
import numpy as np
import propagation_param as p
import random
from Vehicle import Vehicle
import matplotlib.pyplot as plt
import mplcursors
from progress.bar import IncrementalBar
from itertools import combinations

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
        # distances.append(distance)
        if distance == 0:
            distance = 1
        if distance < MAX_DISTANCE:
            received_power = power_dict.get(distance)
        else:
            received_power = POWER_BELOW_MTL
        meets_minimum_trigger_level = True
        if True:  # This is to test power being too high, and meeting the MTL, which may be an unreliable way to
            # accumulate unsuccessful decodes
            # if received_power > -3.0:
            #     meets_minimum_trigger_level = False
            #     false_from_high_power += 1
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
            # if (vehicles[j].message_start_opportunity >=
            #     (vehicles[vehicles[j].transmissions[k]["receiver_ID"]].message_start_opportunity - 9)) and \
            #         (vehicles[j].message_start_opportunity <=
            #          (vehicles[vehicles[j].transmissions[k]["receiver_ID"]].message_start_opportunity + 8)):
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
            # vehicle_combos = list(combinations(vehicle_ids_involved, 2))
            # for k in range(len(vehicle_combos)):
            #     collision_distances.append(vehicles[vehicle_combos[k][0]].transmissions[vehicle_combos[k][1]]["distance"])
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
                            mso + 8)):  # that is not part of the collision (and not switching modes) FIXME +9 not +8?
                transmissions = []
                # compare to each vehicle that was part of the collision
                for l in range(len(collision_objects[j]["vehicles_involved"])):
                    transmissions.append(collision_objects[j]["vehicles_involved"][l].transmissions[k])
                transmissions.sort(key=lambda transmission: transmission["distance"])
                if transmissions[0]["received_power"] >= p.receiver_min_trigger_level:
                    # if not filtered_mso_added:
                    #     filtered_mso_collisions.append(mso)
                    #     filtered_mso_added = True
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
            mso = vehicles[j].full_mso_range()  # calculate MSO
            false_from_high_power, false_from_minimum_trigger_level, false_by_chance = \
                init_transmissions(vehicles, false_from_high_power,
                                   false_from_minimum_trigger_level, false_by_chance, j, distances)
            vehicles[j].frame += 1  # increase frame
            msos.append(mso)  # record MSO
            seconds[i].append(mso)  # record MSO in a way that helps with collisions
        # copy = seconds[i].copy()
        # copy.sort()
        # print(copy)
        false_from_closeness += evaluate_switching_modes(vehicles)
        false_from_message_overlap += evaluate_consecutive_msos(vehicles)
        false_from_collision += evaluate_collisions(seconds, vehicles, i, collision_objects, vehicles_per_collision,
                                                    mso_collisions, collisions_in_time, filtered_mso_collisions,
                                                    collision_distances)
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

    # mso_collisions.sort()
    # filtered_mso_collisions.sort()
    # if mso_collisions == filtered_mso_collisions:
    #     print("they are the same")

    plot_type = "svg"
    # plt.figure(1)
    # plt.hist(msos, bins=3200)
    # mplcursors.cursor()
    # plt.title("Occurrences of Specific MSOs")
    # plt.xlabel("MSO")
    # plt.ylabel("Occurrences of MSO")
    # plt.savefig(("{}V_{}fig1_main_sim." + plot_type).format(p.num_vehicles, ERP_TYPE))
    #
    # plt.figure(2)
    # plt.hist(mso_collisions, bins=3200)
    # mplcursors.cursor()
    # plt.title("Occurrences of MSO Collisions on a Specific MSO")
    # plt.xlabel("MSO")
    # plt.ylabel("Occurrences of Collisions on MSO")
    # plt.savefig(("{}V_{}fig2_main_sim." + plot_type).format(p.num_vehicles, ERP_TYPE))
    #
    # plt.figure(3)
    # plt.bar(x, counts)
    # plt.xticks(x)
    # mplcursors.cursor()
    # plt.title("MSO Collisions by Number of Vehicles Involved")
    # plt.xlabel("Vehicles per Collision")
    # plt.ylabel("Number of Collisions")
    # plt.savefig(("{}V_{}fig3_main_sim." + plot_type).format(p.num_vehicles, ERP_TYPE))
    #
    # plt.figure(4)
    # plt.hist(collisions_in_time, bins=p.num_seconds)
    # mplcursors.cursor()
    # plt.title("Occurrences of Collisions in Time")
    # plt.xlabel("Second")
    # plt.ylabel("Number of Collisions")
    # plt.savefig(("{}V_{}fig4_main_sim." + plot_type).format(p.num_vehicles, ERP_TYPE))
    #
    plt.rcParams.update({"font.size": 15})
    fig = plt.figure(5)
    circle = plt.Circle((0, 0), p.radius_of_area, edgecolor="k", facecolor="none")
    ax = fig.add_subplot(111)
    plt.gca().add_artist(circle)
    # plt.title("2D UAS Positions")

    # print(overall_collision_objects)
    grays = ["Gray", "Silver", "Gainsboro", "DimGray", "LightGray"]
    point_size = 10
    plot_collisions = False
    legend_not_added = True
    if p.num_vehicles < 30:
        plot_collisions = True
        plot_collisions = False
    for i in range(p.num_vehicles):
        north_vals = []
        east_vals = []
        for j in range(1, p.num_seconds):
            north_vals.append(p.vehicle_positions[i][j][0])
            east_vals.append(p.vehicle_positions[i][j][1])
        gray = grays[i % len(grays)]
        ax.scatter(east_vals, north_vals, c=gray, s=point_size)
        if legend_not_added:
            legend_not_added = False
            ax.scatter(p.vehicle_positions[i][0][1], p.vehicle_positions[i][0][0],
                       marker="v", c="Black", s=point_size * 5, label="UAS Starting Position")
        else:
            ax.scatter(p.vehicle_positions[i][0][1], p.vehicle_positions[i][0][0],
                       marker="v", c="Black", s=point_size * 5)

    if plot_collisions:
        collisions_2d = []
        legend_labels = []
        color_index = 0
        label_index = 0
        colors = ["Red", "DarkOrange", "Yellow", "Lime", "Aqua", "DodgerBlue", "Indigo", "BlueViolet", "Fuchsia"]
        for i in range(num_MSO_collisions):
            legend_labels.append(legend_labels.append("Collision {}".format(i + 1)))
        for i in range(p.num_seconds):
            for j in range(len(overall_collision_objects[i])):
                color = colors[color_index]
                if len(overall_collision_objects[i][j]["vehicles_involved"]) > 1:
                    color_index += 1
                for k in range(len(overall_collision_objects[i][j]["vehicles_involved"])):
                    vehicle_id = overall_collision_objects[i][j]["vehicles_involved"][k].vehicle_identifier
                    if k == 0:
                        label_index += 1
                        collisions_2d.append(ax.scatter(p.vehicle_positions[vehicle_id][i][1],
                                                        p.vehicle_positions[vehicle_id][i][0],
                                                        marker="D", c=color, s=point_size * 2,
                                                        label="Collision {}".format(label_index)))
                    else:
                        ax.scatter(p.vehicle_positions[vehicle_id][i][1],
                                   p.vehicle_positions[vehicle_id][i][0],
                                   marker="D", c=color, s=point_size * 2)

        # plt.legend(collisions_2d, legend_labels, fontsize=8)
        plt.legend(fontsize=8, bbox_to_anchor=(1, 0.7))
        plt.title("2D UAS Positions With MSO Collisions")
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    plt.savefig(("{}V_{}fig5_main_sim." + plot_type).format(p.num_vehicles, ERP_TYPE))

    # plt.figure(6)
    # plt.hist(filtered_mso_collisions, bins=3200)
    # mplcursors.cursor()
    # plt.title("Occurrences of MSO Collisions--Filtered")
    # plt.xlabel("MSO")
    # plt.ylabel("Occurrences of Collisions on MSO")
    # plt.savefig(("{}V_{}fig6." + plot_type).format(p.num_vehicles, ERP_TYPE))

    # plt.figure(7)
    # plt.hist(collision_distances)
    # mplcursors.cursor()
    # plt.title("Distances Between Vehicles \nInvolved in MSO Collisions")
    # plt.xlabel("Distance")
    # plt.ylabel("Occurrences of Distance")
    # plt.savefig(("{}V_{}fig7." + plot_type).format(p.num_vehicles, ERP_TYPE))

    # plt.figure(8)
    # plt.hist(distances)
    # mplcursors.cursor()
    # plt.title("Distances Between Vehicles")
    # plt.xlabel("Distance")
    # plt.ylabel("Occurrences of Distance")
    # plt.savefig(("{}V_{}fig8." + plot_type).format(p.num_vehicles, ERP_TYPE))

    # plt.rcParams.update({"font.size": 15})
    # fig = plt.figure(9)
    # ax = fig.add_subplot(111, projection="3d")
    # plt.title("3D UAS Positions")
    # for i in range(p.num_vehicles):
    #     north_vals = []
    #     east_vals = []
    #     down_vals = []
    #     for j in range(p.num_seconds):
    #         north_vals.append(p.vehicle_positions[i][j][0])
    #         east_vals.append(p.vehicle_positions[i][j][1])
    #         down_vals.append(-p.vehicle_positions[i][j][2])
    #     ax.scatter(north_vals, east_vals, down_vals, c="gray", s=point_size)
    #
    # if plot_collisions:
    #     colors = ["Aqua", "DeepPink", "Fuchsia", "Lime", "DodgerBlue"]
    #     for i in range(p.num_seconds):
    #         for j in range(len(overall_collision_objects[i])):
    #             color = colors[i % len(colors)]
    #             for k in range(len(overall_collision_objects[i][j]["vehicles_involved"])):
    #                 vehicle_id = overall_collision_objects[i][j]["vehicles_involved"][k].vehicle_identifier
    #                 ax.scatter(p.vehicle_positions[vehicle_id][i][0],
    #                            p.vehicle_positions[vehicle_id][i][1],
    #                            -p.vehicle_positions[vehicle_id][i][2],
    #                            marker="D", c=color, s=point_size * 4)
    #
    #     plt.legend(collisions_2d, legend_labels, loc="center left", fontsize=8)
    #     plt.title("3D UAS Positions With MSO Collisions")
    #
    # ax.set_xlabel("X (m)")
    # ax.set_ylabel("Y (m)")
    # ax.set_zlabel("Z (m)")
    # plt.savefig(("{}V_{}fig9_main_sim." + plot_type).format(p.num_vehicles, ERP_TYPE))

    plt.show()
    # with open("collisions_per_sec{}V_{}S_main_sim".format(p.num_vehicles, p.num_seconds), "w", newline="") as myfile:
    #     wr = csv.writer(myfile)
    #     wr.writerow(collisions_per_second)


if __name__ == "__main__":
    start = time.time()
    main()
    end = time.time()
    print("Total runtime:", end - start)
