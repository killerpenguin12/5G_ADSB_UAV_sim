import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
import pdb
import scipy as sp
import scipy.optimize

import numpy.polynomial.polynomial as poly

from scipy.optimize import curve_fit

def func(x, a, b, c):
    return a * np.exp(-b * x) + c

def fit_linear(t, y):
    # pdb.set_trace()
    A_mat = np.vstack((np.ones(y.shape),t)).T
    x = np.polyfit(t,y,10)
    return x



plt.rcParams.update({'font.size': 15})

monteType = np.dtype([('range',np.float64),
    ('velocity',np.float64),
    ('vehicles',np.int32),
    ('collisions',np.int64)])



# allFiles = ['./monte_carlo_gps/WorstCase/monte_carlo2.log']
allFiles = ['./monte_carlo_gps/BestCase/monte_carlo2.log']

# allFiles = ['./monte_carlo_gps/RVO/monte_carlo.log']
# allFiles = ['./monte_carlo_gps/Buffer/monte_carlo2.log']
# allFiles = ['./monte_carlo_gps/UncertaintyVO/monte_carlo2.log']
# allFiles = ['./monte_carlo_gps/Both/monte_carlo2.log']

# allFiles = ['./monte_carlo_radar/RVO/monte_carlo.log']
# allFiles = ['./monte_carlo_radar/Buffer/monte_carlo.log']
# allFiles = ['./monte_carlo_radar/UncertaintyVO/monte_carlo.log']
# allFiles = ['./monte_carlo_radar/Both/monte_carlo.log']


# allFiles = ['./monte_carlo_gps/LongCase/monte_carlo.log','./monte_carlo_gps/LongCase/monte_carlo2.log']


allFiles = ['/tmp/monte_carlo.log']

# allFiles = ['./velupdate_gps/UVO/monte_carlo.log']
# allFiles = ['./velupdate_gps/Both/monte_carlo.log']
#
# allFiles = ['./velupdate_gps/Long/monte_carlo_1_1_2.log',
#     './velupdate_gps/Long/monte_carlo_1_1_3.log']

zeroCollisions = np.array([])
zeroRange = np.array([])
zeroSpeed = np.array([])
zeroVehicles = np.array([])

notZeroCollisions = np.array([])
notZeroRange = np.array([])
notZeroSpeed = np.array([])
notZeroVehicles = np.array([])

for file in allFiles:
    monteData = np.fromfile(file, dtype=monteType)
    for i in range(len(monteData['collisions'])):
        # if monteData['range'][i] > 50:
        #     continue
        if monteData['collisions'][i] == 0:
            zeroCollisions = np.append(zeroCollisions, monteData['collisions'][i])
            zeroRange = np.append(zeroRange, monteData['range'][i])
            zeroSpeed = np.append(zeroSpeed, monteData['velocity'][i])
            zeroVehicles = np.append(zeroVehicles, monteData['vehicles'][i])
        else:
            notZeroCollisions = np.append(notZeroCollisions, monteData['collisions'][i])
            notZeroRange = np.append(notZeroRange, monteData['range'][i])
            notZeroSpeed = np.append(notZeroSpeed, monteData['velocity'][i])
            notZeroVehicles = np.append(notZeroVehicles, monteData['vehicles'][i])

print(np.max(monteData['collisions']))

## A = fit_linear(monteData['range'], 1.0*monteData['collisions']/monteData['vehicles'])
## # print(A)
## ffit = np.poly1d(A)
## plt.plot(x_new, ffit(x_new),linewidth=2,color='black')


# pdb.set_trace()
x_new = np.linspace(11, 50, num=1000)
popt, pcov = curve_fit(func, monteData['range']-11, 1.0*monteData['collisions']/monteData['vehicles']+1)
print(popt,pcov)

fig = plt.figure()
plt.gcf().subplots_adjust(bottom=0.15)
plt.gcf().subplots_adjust(left=0.15)
fig.subplots_adjust(hspace=0.5)

ax = fig.add_subplot(111)
ax.title.set_text(" ")
# plt.scatter(zeroRange, zeroCollisions, color='red', s=20)
# plt.scatter(notZeroRange, notZeroCollisions, s=20)
plt.scatter(zeroRange, 1.0*zeroCollisions/zeroVehicles, color='red', s=20, marker="*", label='Zero Collisions')
plt.scatter(notZeroRange, 1.0*notZeroCollisions/notZeroVehicles, s=20, marker="o", label='Non-Zero Collisions')
# plt.plot(x_new, func(x_new-11, *popt)-1, color="black", label='$y = %0.2f e^{-%0.2f t}$' % (popt[0],popt[1]))
# plt.plot(x_new, np.ones_like(x_new)*0.0, color='black')
# xy = (42.04, 0.0)
# plt.scatter(*xy, color='black',s=75)
# ax.annotate('(%s, %s)' % xy, xy=xy, xytext=(30,-0.2))
# ax.plot([80,80],[-0.05,0.05],linewidth=3.0,color='black')
ax.set_ylim([-0.5,2.5])
# ax.set_xlim([10,50])
# start, end = ax.get_xlim()
# ax.xaxis.set_ticks(np.arange(0, 220, 20))
ax.set_xlabel('Collision Range (m)')
ax.set_ylabel('# Collisions / Vehicle')
ax.legend()


# fig = plt.figure()
# fig.subplots_adjust(hspace=0.5)
# plt.gcf().subplots_adjust(left=0.15)
#
# ax = fig.add_subplot(211)
# plt.scatter(zeroSpeed, 1.0*zeroCollisions/zeroVehicles, color='red', s=20, marker="*", label='Zero Collisions')
# plt.scatter(notZeroSpeed, 1.0*notZeroCollisions/notZeroVehicles, s=20, marker="o", label='Non-Zero Collisions')
# ax.set_xlabel('Max Velocity (m/s)')
# ax.set_ylabel('# Collisions / Vehicle')
#
#
# ax = fig.add_subplot(212)
# plt.scatter(zeroVehicles, 1.0*zeroCollisions/zeroVehicles, color='red', s=20, marker="*", label='Zero Collisions')
# plt.scatter(notZeroVehicles, 1.0*notZeroCollisions/notZeroVehicles, s=20, marker="o", label='Non-Zero Collisions')
# # plt.scatter(notZeroVehicles, 1.0*notZeroCollisions/notZeroVehicles)
# # plt.scatter(zeroVehicles, 1.0*zeroCollisions/zeroVehicles, color='red')
# ax.set_xlabel('Number of Vehicles')
# ax.set_ylabel('# Collisions / Vehicle')


# fig = plt.figure()
# plt.gcf().subplots_adjust(bottom=0.15)
# ax = fig.add_subplot(111)
# ax.scatter(zeroSpeed, zeroRange, color='red', marker="*")
# ax.plot([5,10],[25,90],linewidth=2.0,color='black')
# ax.scatter(notZeroSpeed, notZeroRange)
# ax.set_xlabel('Max Velocity (m/s)')
# ax.set_ylabel('Range (m)')

# fig2 = plt.figure()
# ax = fig2.add_subplot(111, projection='3d')
# ax.title.set_text(" ")
# ax.scatter(1.0*zeroCollisions/zeroVehicles, zeroSpeed, zeroRange, color='red', marker="*")
# ax.scatter(1.0*notZeroCollisions/notZeroVehicles, notZeroSpeed, notZeroRange)
# ax.set_xlabel('# Collisions / Vehicle')
# ax.set_ylabel('Max Velocity')
# ax.set_zlabel('Range')


plt.show()
