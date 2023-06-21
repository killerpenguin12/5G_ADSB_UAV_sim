# Feb 1, 2021
# These are the results plots we put in our SciTech 2021 conference paper.

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

vehicle_amounts = [1000, 5000, 10000]
vehicle_densities = [1000.0 / (np.pi * 37.04**2.0),
                     5000.0 / (np.pi * 37.04**2.0),
                     10000.0 / (np.pi * 37.04**2.0)]
hundredthW_marker = "o"
tenthW_marker = "v"
oneW_marker = "s"
tenW_marker = "X"#(4, 1, 45)#"*"
method1 = Line2D([0], [0], color="dimgray", label="Method 1", linestyle="--")
method2 = Line2D([0], [0], color="dimgray", label="Method 2", linestyle="-")
hundredthW = Line2D([0], [0], marker=hundredthW_marker, color="w", label="0.01 W", markerfacecolor="tab:blue", markersize=9)
tenthW = Line2D([0], [0], marker=tenthW_marker, color="w", label="0.1 W", markerfacecolor="tab:green", markersize=9)
oneW = Line2D([0], [0], marker=oneW_marker, color="w", label="1 W", markerfacecolor="tab:orange", markersize=9)
tenW = Line2D([0], [0], marker=tenW_marker, color="w", label="10 W", markerfacecolor="tab:red", markersize=9)
legend_elements = [method1, method2, hundredthW, tenthW, oneW, tenW]
x_limits = (-vehicle_densities[2]/20.0, vehicle_densities[2] + vehicle_densities[2]/20.0)
y_limits = (-0.05, 1.05)
x_label = "UAS per $\mathregular{km^{2}}$"
y_label = "Successful Decode Probability"
secondary_x_axis_label = "Number of UAS"


def vehicle_amount_to_vehicle_density(vehicle_amount):
    return vehicle_amount / (np.pi * 37.04**2.0)


def vehicle_density_to_vehicle_amount(vehicle_density):
    return vehicle_density * (np.pi * 37.04**2.0)


# Standard model
# List for 1000, 5000, then 10000
standard_tenW_normal = [0.567435969302636, 0.11411972794558911, 0.021684088742207508]
standard_oneW_normal = [0.8032585585585585, 0.5820075521771021, 0.5062778317831783]
standard_tenthW_normal = [0.9793037037037037, 0.9596963686070548, 0.9529896903023636]
standard_hundredthW_normal = [0.99812999666333, 0.9965323544708942, 0.9962006490649065]
standard_tenW_better = [0.567435969302636, 0.11411972794558911, 0.021684088742207508]
standard_oneW_better = [0.6286391229739298, 0.22117323056441895, 0.08695857499136506]
standard_tenthW_better = [0.7256269590396492, 0.4675941570522716, 0.38106153855377733]
standard_hundredthW_better = [0.7370085686666479, 0.5193001734444078, 0.474203673377645]

plt.rcParams.update({"font.size": 15})
fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.plot(vehicle_densities, standard_hundredthW_normal, marker=hundredthW_marker, linestyle="--", color="tab:blue")
ax.plot(vehicle_densities, standard_hundredthW_better, marker=hundredthW_marker, linestyle="-", color="tab:blue")
ax.plot(vehicle_densities, standard_tenthW_normal, marker=tenthW_marker, linestyle="--", color="tab:green")
ax.plot(vehicle_densities, standard_tenthW_better, marker=tenthW_marker, linestyle="-", color="tab:green")
ax.plot(vehicle_densities, standard_oneW_normal, marker=oneW_marker, linestyle="--", color="tab:orange")
ax.plot(vehicle_densities, standard_oneW_better, marker=oneW_marker, linestyle="-", color="tab:orange")
ax.plot(vehicle_densities, standard_tenW_normal, marker=tenW_marker, linestyle="--", color="tab:red")
ax.plot(vehicle_densities, standard_tenW_better, marker=tenW_marker, linestyle="-", color="tab:red")
second_axis = ax.secondary_xaxis("top", functions=(vehicle_density_to_vehicle_amount, vehicle_amount_to_vehicle_density))
second_axis.set_xlabel(secondary_x_axis_label)
ax.legend(handles=legend_elements, loc="lower left", fontsize=11)
ax.set(xlim=x_limits, ylim=y_limits)
ax.grid(axis="y")
plt.xlabel(x_label)
plt.ylabel(y_label)


# Message enhanced model
message_tenW_normal = [0.7183581581581582, 0.20550468360338736, 0.04404701436810343]
message_oneW_normal = [0.8914091091091091, 0.676176473961459, 0.5523971267126713]
message_tenthW_normal = [0.993813046379713, 0.9872928318997133, 0.979921834516785]
message_hundredthW_normal = [0.9995111444778112, 0.9994422217776888, 0.9993638760542721]
message_tenW_better = [0.7183581581581582, 0.20550468360338736, 0.04404701436810343]
message_oneW_better = [0.79502839776496, 0.39663400148633055, 0.17224708232110963]
message_tenthW_better = [0.9179788859428648, 0.8321399261036353, 0.7356505640406673]
message_hundredthW_better = [0.9312488855102251, 0.9226784016789094, 0.9119661121980511]

plt.rcParams.update({"font.size": 15})
fig = plt.figure(2)
ax = fig.add_subplot(111)
ax.plot(vehicle_densities, message_hundredthW_normal, marker=hundredthW_marker, linestyle="--", color="tab:blue")
ax.plot(vehicle_densities, message_hundredthW_better, marker=hundredthW_marker, linestyle="-", color="tab:blue")
ax.plot(vehicle_densities, message_tenthW_normal, marker=tenthW_marker, linestyle="--", color="tab:green")
ax.plot(vehicle_densities, message_tenthW_better, marker=tenthW_marker, linestyle="-", color="tab:green")
ax.plot(vehicle_densities, message_oneW_normal, marker=oneW_marker, linestyle="--", color="tab:orange")
ax.plot(vehicle_densities, message_oneW_better, marker=oneW_marker, linestyle="-", color="tab:orange")
ax.plot(vehicle_densities, message_tenW_normal, marker=tenW_marker, linestyle="--", color="tab:red")
ax.plot(vehicle_densities, message_tenW_better, marker=tenW_marker, linestyle="-", color="tab:red")
second_axis = ax.secondary_xaxis("top", functions=(vehicle_density_to_vehicle_amount, vehicle_amount_to_vehicle_density))
second_axis.set_xlabel(secondary_x_axis_label)
ax.legend(handles=legend_elements, loc="lower left", fontsize=11)
ax.set(xlim=x_limits, ylim=y_limits)
ax.grid(axis="y")
plt.xlabel(x_label)
plt.ylabel(y_label)


# Collision enhanced model
collision_tenW_normal = [0.6172038371705038, 0.1941452597186104, 0.08425430109677634]
collision_oneW_normal = [0.8176369035702369, 0.6149636687337467, 0.5420871803847052]
collision_tenthW_normal = [0.979578044711378, 0.9606470920850837, 0.9546055112177885]
collision_hundredthW_normal = [0.9981323323323323, 0.9965392918583716, 0.996214100410041]
collision_tenW_better = [0.6172038371705038, 0.1941452597186104, 0.08425430109677634]
collision_oneW_better = [0.6557790827841234, 0.28257889931427427, 0.15318087729116903]
collision_tenthW_better = [0.7292639274835735, 0.48015309323832445, 0.40233546161019107]
collision_hundredthW_better = [0.7373370498634457, 0.5202618637121097, 0.47606521972613536]

plt.rcParams.update({"font.size": 15})
fig = plt.figure(3)
ax = fig.add_subplot(111)
ax.plot(vehicle_densities, collision_hundredthW_normal, marker=hundredthW_marker, linestyle="--", color="tab:blue")
ax.plot(vehicle_densities, collision_hundredthW_better, marker=hundredthW_marker, linestyle="-", color="tab:blue")
ax.plot(vehicle_densities, collision_tenthW_normal, marker=tenthW_marker, linestyle="--", color="tab:green")
ax.plot(vehicle_densities, collision_tenthW_better, marker=tenthW_marker, linestyle="-", color="tab:green")
ax.plot(vehicle_densities, collision_oneW_normal, marker=oneW_marker, linestyle="--", color="tab:orange")
ax.plot(vehicle_densities, collision_oneW_better, marker=oneW_marker, linestyle="-", color="tab:orange")
ax.plot(vehicle_densities, collision_tenW_normal, marker=tenW_marker, linestyle="--", color="tab:red")
ax.plot(vehicle_densities, collision_tenW_better, marker=tenW_marker, linestyle="-", color="tab:red")
second_axis = ax.secondary_xaxis("top", functions=(vehicle_density_to_vehicle_amount, vehicle_amount_to_vehicle_density))
second_axis.set_xlabel(secondary_x_axis_label)
ax.legend(handles=legend_elements, loc="lower left", fontsize=11)
ax.set(xlim=x_limits, ylim=y_limits)
ax.grid(axis="y")
plt.xlabel(x_label)
plt.ylabel(y_label)


# Dual enhanced model
dual_tenW_normal = [0.7814887220553888, 0.34788944855637793, 0.16551822382238224]
dual_oneW_normal = [0.9096394728061394, 0.7344313716076549, 0.6211463102976964]
dual_tenthW_normal = [0.9941633633633634, 0.9889531319597252, 0.9829805417208387]
dual_hundredthW_normal = [0.9995139139139139, 0.9994545402413816, 0.9993892845951262]
dual_tenW_better = [0.7814887220553888, 0.34788944855637793, 0.16551822382238224]
dual_oneW_better = [0.8294392661688519, 0.5051777657909204, 0.29938508946257825]
dual_tenthW_better = [0.9226230761268542, 0.854072278659628, 0.7759215501938745]
dual_hundredthW_better = [0.9316383703578568, 0.9243860397032774, 0.9154824279251698]

plt.rcParams.update({"font.size": 15})
fig = plt.figure(4)
ax = fig.add_subplot(111)
ax.plot(vehicle_densities, dual_hundredthW_normal, marker=hundredthW_marker, linestyle="--", color="tab:blue")
ax.plot(vehicle_densities, dual_hundredthW_better, marker=hundredthW_marker, linestyle="-", color="tab:blue")
ax.plot(vehicle_densities, dual_tenthW_normal, marker=tenthW_marker, linestyle="--", color="tab:green")
ax.plot(vehicle_densities, dual_tenthW_better, marker=tenthW_marker, linestyle="-", color="tab:green")
ax.plot(vehicle_densities, dual_oneW_normal, marker=oneW_marker, linestyle="--", color="tab:orange")
ax.plot(vehicle_densities, dual_oneW_better, marker=oneW_marker, linestyle="-", color="tab:orange")
ax.plot(vehicle_densities, dual_tenW_normal, marker=tenW_marker, linestyle="--", color="tab:red")
ax.plot(vehicle_densities, dual_tenW_better, marker=tenW_marker, linestyle="-", color="tab:red")
second_axis = ax.secondary_xaxis("top", functions=(vehicle_density_to_vehicle_amount, vehicle_amount_to_vehicle_density))
second_axis.set_xlabel(secondary_x_axis_label)
ax.legend(handles=legend_elements, loc="lower left", fontsize=11)
ax.set(xlim=x_limits, ylim=y_limits)
ax.grid(axis="y")
plt.xlabel(x_label)
plt.ylabel(y_label)

plt.show()
