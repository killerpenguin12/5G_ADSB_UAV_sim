# Feb 1, 2021
# This was an attempt at making some figures with more information.
# The version that we eventually used is in decode_probability_plot.py

import numpy as np
import matplotlib.pyplot as plt

n_groups = 4
probabilities_standard = (0.99, 0.97, 0.76, 0.57)
probabilities_enhanced1 = (0.99, 0.98, 0.84, 0.72)
probabilities_enhanced2 = (0.99, 0.97, 0.80, 0.62)
probabilities_enhanced3 = (0.99, 0.99, 0.89, 0.78)

fig1, ax1 = plt.subplots()
index = np.arange(n_groups)
bar_width = 0.2
opacity = 1

rects11 = plt.bar(index, probabilities_standard, bar_width,
                  alpha=opacity,
                  color='Red',
                  label='Standard')

rects21 = plt.bar(index + bar_width, probabilities_enhanced1, bar_width,
                  alpha=opacity,
                  color='DarkOrange',
                  label='Enhanced 1')

rects31 = plt.bar(index + 2 * bar_width, probabilities_enhanced2, bar_width,
                  alpha=opacity,
                  color='Lime',
                  label='Enhanced 2')

rects41 = plt.bar(index + 3 * bar_width, probabilities_enhanced3, bar_width,
                  alpha=opacity,
                  color='DodgerBlue',
                  label='Enhanced 3')

plt.xlabel('Transmit Power')
plt.ylabel('Decode Probability')
plt.title('1000 UAS Decode Probabilities')
plt.xticks(index + bar_width, ('0.01', '0.1', '1.0', '10.0'))
plt.legend()

n_groups = 4
probabilities_standard = (0.99, 0.93, 0.52, 0.11)
probabilities_enhanced1 = (0.99, 0.94, 0.57, 0.21)
probabilities_enhanced2 = (0.99, 0.95, 0.59, 0.19)
probabilities_enhanced3 = (0.99, 0.97, 0.69, 0.35)

fig1, ax1 = plt.subplots()
index = np.arange(n_groups)
bar_width = 0.2
opacity = 1

rects12 = plt.bar(index, probabilities_standard, bar_width,
                  alpha=opacity,
                  color='Red',
                  label='Standard')

rects22 = plt.bar(index + bar_width, probabilities_enhanced1, bar_width,
                  alpha=opacity,
                  color='DarkOrange',
                  label='Enhanced 1')

rects32 = plt.bar(index + 2 * bar_width, probabilities_enhanced2, bar_width,
                  alpha=opacity,
                  color='Lime',
                  label='Enhanced 2')

rects42 = plt.bar(index + 3 * bar_width, probabilities_enhanced3, bar_width,
                  alpha=opacity,
                  color='DodgerBlue',
                  label='Enhanced 3')

plt.xlabel('Transmit Power')
plt.ylabel('Decode Probability')
plt.title('5000 UAS Decode Probabilities')
plt.xticks(index + bar_width, ('0.01', '0.1', '1.0', '10.0'))
plt.legend()

n_groups = 4
probabilities_standard = (0.99, 0.93, 0.47, 0.02)
probabilities_enhanced1 = (0.99, 0.93, 0.48, 0.04)
probabilities_enhanced2 = (0.99, 0.94, 0.53, 0.08)
probabilities_enhanced3 = (0.99, 0.96, 0.59, 0.17)

fig1, ax1 = plt.subplots()
index = np.arange(n_groups)
bar_width = 0.2
opacity = 1

rects13 = plt.bar(index, probabilities_standard, bar_width,
                  alpha=opacity,
                  color='Red',
                  label='Standard')

rects23 = plt.bar(index + bar_width, probabilities_enhanced1, bar_width,
                  alpha=opacity,
                  color='DarkOrange',
                  label='Enhanced 1')

rects33 = plt.bar(index + 2 * bar_width, probabilities_enhanced2, bar_width,
                  alpha=opacity,
                  color='Lime',
                  label='Enhanced 2')

rects43 = plt.bar(index + 3 * bar_width, probabilities_enhanced3, bar_width,
                  alpha=opacity,
                  color='DodgerBlue',
                  label='Enhanced 3')

plt.xlabel('Transmit Power')
plt.ylabel('Decode Probability')
plt.title('10000 UAS Decode Probabilities')
plt.xticks(index + bar_width, ('0.01', '0.1', '1.0', '10.0'))
plt.legend()

plt.tight_layout()
plt.show()
