# Feb 1, 2021
# This is a modification of decode_probability_plot.py that I used to make some slides.

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
# method1 = Line2D([0], [0], color="dimgray", label="Method 1", linestyle="--")
# method2 = Line2D([0], [0], color="dimgray", label="Method 2", linestyle="-")
hundredthW = Line2D([0], [0], marker=hundredthW_marker, color="w", label="0.01 W", markerfacecolor="tab:blue", markersize=9)
tenthW = Line2D([0], [0], marker=tenthW_marker, color="w", label="0.1 W", markerfacecolor="tab:green", markersize=9)
oneW = Line2D([0], [0], marker=oneW_marker, color="w", label="1 W", markerfacecolor="tab:orange", markersize=9)
tenW = Line2D([0], [0], marker=tenW_marker, color="w", label="10 W", markerfacecolor="tab:red", markersize=9)
legend_elements = [hundredthW, tenthW, oneW, tenW]
x_limits = (-vehicle_densities[2]/20.0, vehicle_densities[2] + vehicle_densities[2]/20.0)
y_limits = (-0.05, 1.05)
x_label = "UAS per $\mathregular{km^{2}}$"
y_label = "Successful Decode Probability"
secondary_x_axis_label = "Number of UAS"


def vehicle_amount_to_vehicle_density(vehicle_amount):
    return vehicle_amount / (np.pi * 37.04**2.0)


def vehicle_density_to_vehicle_amount(vehicle_density):
    return vehicle_density * (np.pi * 37.04**2.0)


standard_tenW_normal = [0.567435969302636, 0.11411972794558911, 0.021684088742207508]
standard_oneW_normal = [0.8032585585585585, 0.5820075521771021, 0.5062778317831783]
standard_tenthW_normal = [0.9793037037037037, 0.9596963686070548, 0.9529896903023636]
standard_hundredthW_normal = [0.99812999666333, 0.9965323544708942, 0.9962006490649065]

plt.rcParams.update({"font.size": 15})
fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.plot(vehicle_densities, standard_hundredthW_normal, marker=hundredthW_marker, linestyle="-", color="tab:blue")
# ax.plot(vehicle_densities, standard_hundredthW_better, marker=hundredthW_marker, linestyle="-", color="tab:blue")
ax.plot(vehicle_densities, standard_tenthW_normal, marker=tenthW_marker, linestyle="-", color="tab:green")
# ax.plot(vehicle_densities, standard_tenthW_better, marker=tenthW_marker, linestyle="-", color="tab:green")
ax.plot(vehicle_densities, standard_oneW_normal, marker=oneW_marker, linestyle="-", color="tab:orange")
# ax.plot(vehicle_densities, standard_oneW_better, marker=oneW_marker, linestyle="-", color="tab:orange")
ax.plot(vehicle_densities, standard_tenW_normal, marker=tenW_marker, linestyle="-", color="tab:red")
# ax.plot(vehicle_densities, standard_tenW_better, marker=tenW_marker, linestyle="-", color="tab:red")
second_axis = ax.secondary_xaxis("top", functions=(vehicle_density_to_vehicle_amount, vehicle_amount_to_vehicle_density))
second_axis.set_xlabel(secondary_x_axis_label)
ax.legend(handles=legend_elements, loc="lower left", fontsize=11)
ax.set(xlim=x_limits, ylim=y_limits)
ax.grid(axis="y")
plt.xlabel(x_label)
plt.ylabel(y_label)


dual_tenW_normal = [0.7814887220553888, 0.34788944855637793, 0.16551822382238224]
dual_oneW_normal = [0.9096394728061394, 0.7344313716076549, 0.6211463102976964]
dual_tenthW_normal = [0.9941633633633634, 0.9889531319597252, 0.9829805417208387]
dual_hundredthW_normal = [0.9995139139139139, 0.9994545402413816, 0.9993892845951262]

plt.rcParams.update({"font.size": 15})
fig = plt.figure(2)
ax = fig.add_subplot(111)
ax.plot(vehicle_densities, dual_hundredthW_normal, marker=hundredthW_marker, linestyle="-", color="tab:blue")
# ax.plot(vehicle_densities, dual_hundredthW_better, marker=hundredthW_marker, linestyle="-", color="tab:blue")
ax.plot(vehicle_densities, dual_tenthW_normal, marker=tenthW_marker, linestyle="-", color="tab:green")
# ax.plot(vehicle_densities, dual_tenthW_better, marker=tenthW_marker, linestyle="-", color="tab:green")
ax.plot(vehicle_densities, dual_oneW_normal, marker=oneW_marker, linestyle="-", color="tab:orange")
# ax.plot(vehicle_densities, dual_oneW_better, marker=oneW_marker, linestyle="-", color="tab:orange")
ax.plot(vehicle_densities, dual_tenW_normal, marker=tenW_marker, linestyle="-", color="tab:red")
# ax.plot(vehicle_densities, dual_tenW_better, marker=tenW_marker, linestyle="-", color="tab:red")
second_axis = ax.secondary_xaxis("top", functions=(vehicle_density_to_vehicle_amount, vehicle_amount_to_vehicle_density))
second_axis.set_xlabel(secondary_x_axis_label)
ax.legend(handles=legend_elements, loc="lower left", fontsize=11)
ax.set(xlim=x_limits, ylim=y_limits)
ax.grid(axis="y")
plt.xlabel(x_label)
plt.ylabel(y_label)

plt.show()