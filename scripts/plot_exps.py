import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches


def func(x, a, b, c):
    return a * np.exp(-b * x) + c

x = np.linspace(0,19,50)

# RVO, Buffer, Uncertainty, Both, WorstCase, BestCase

# GPS Original
# a = [1.65645071, 0.76535239, 0.56129471, 0.60704809, 2.26778512, 1.58116742]
# b = [0.20192778, 0.1311208, 0.15400636, 0.2120955, 0.10050204, 0.20587041]
# c = [1.04543, 0.98426655, 0.99961105, 1.01353852, 1.13452438, 1.01058215]

# GPS
a = [1.65645071, 0.76535239, 1.33456417, 0.82873833, 2.26778512, 1.58116742]
b = [0.20192778, 0.1311208, 0.26737397, 0.21867123, 0.10050204, 0.20587041]
c = [1.04543, 0.98426655, 1.02109662, 1.00287507, 1.13452438, 1.01058215]

# Radar
# a = [8.82846179, 2.28175189, 1.01808223, 2.49164754]
# b = [0.12345462, 0.0856767, 0.08034029, 0.12796625]
# c = [1.08760317, 0.94923484, 1.02932022, 1.03734641]


plt.rcParams.update({'font.size': 15})

fig = plt.figure()
plt.gcf().subplots_adjust(bottom=0.15)
ax = fig.add_subplot(111)
plt.scatter(x+11,func(x,a[4],b[4],c[4])-1, label='RVO Worst Case', s=10)
plt.scatter(x+11,func(x,a[5],b[5],c[5])-1, label='RVO Best Case', s=10, marker='^')
plt.scatter(x+11,func(x,a[0],b[0],c[0])-1, label='KF-RVO', s=30, marker="1")
plt.scatter(x+11,func(x,a[1],b[1],c[1])-1, label='PF', s=30, marker="+")
plt.scatter(x+11,func(x,a[2],b[2],c[2])-1, label='UVO', s=30, marker="*")
plt.scatter(x+11,func(x,a[3],b[3],c[3])-1, label='UVO-PF', s=10, marker="D")
ax.set_ylim([-0.1,2])
ax.set_xlabel('Collision Range (m)')
ax.set_ylabel('# Collisions / Vehicle')

ax.legend()
plt.show()
