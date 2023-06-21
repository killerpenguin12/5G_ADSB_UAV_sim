# Feb 1 2021
# Code for a figure Tanner originally made that I later worked on. We eventually scrapped it
# for being too simplistic and providing no new information

import matplotlib.pyplot as plt
x = []
y = []
count = 0
for i in range(0, 16385):
    x.append(i * 2.3886)
    if count == 4096:
        count = 0
    y.append(count)
    count += 1

plt.rcParams.update({"font.size": 15})
plt.figure(1)
plt.plot(x, y, c="dimgray")
plt.ylabel('Value of 12 LSBs of Latitude')
plt.xlabel('Lateral Distance (m)')
# plt.show()
# input("")

x = []
y = []
count = 0
for i in range(0, 16385):
    x.append(i * 1.8235)
    if count == 4096:
        count = 0
    y.append(count)
    count += 1

plt.rcParams.update({"font.size": 15})
plt.figure(2)
plt.plot(x, y, c="dimgray")
plt.ylabel('Value of 12 LSBs of Longitude')
plt.xlabel('Longitudinal Distance (m)')
plt.show()
# input("")