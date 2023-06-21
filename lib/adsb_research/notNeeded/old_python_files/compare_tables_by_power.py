# Feb 1 2021
# This was a prototype plot that was later ditched.
# See decode probability plot

import numpy as np
import matplotlib.pyplot as plt

n_groups = 3
probabilities_standard = (0.57, 0.11, 0.02)
probabilities_enhanced1 = (0.72, 0.21, 0.04)
probabilities_enhanced2 = (0.62, 0.19, 0.08)
probabilities_enhanced3 = (0.78, 0.35, 0.17)

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

plt.xlabel('Number of UAS')
plt.ylabel('Decode Probabilities')
plt.title('10 W Decode Probabilities')
plt.xticks(index + bar_width, ('1000', '5000', '10000'))
plt.legend()

n_groups = 3
probabilities_standard = (0.76, 0.52, 0.47)
probabilities_enhanced1 = (0.84, 0.57, 0.48)
probabilities_enhanced2 = (0.80, 0.59, 0.53)
probabilities_enhanced3 = (0.89, 0.69, 0.59)

fig2, ax2 = plt.subplots()
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

plt.xlabel('Number of UAS')
plt.ylabel('Decode Probabilities')
plt.title('1 W Decode Probabilities')
plt.xticks(index + bar_width, ('1000', '5000', '10000'))
plt.legend()

n_groups = 3
probabilities_standard = (0.97, 0.93, 0.93)
probabilities_enhanced1 = (0.98, 0.94, 0.93)
probabilities_enhanced2 = (0.97, 0.95, 0.94)
probabilities_enhanced3 = (0.99, 0.97, 0.96)

fig3, ax3 = plt.subplots()
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

plt.xlabel('Number of UAS')
plt.ylabel('Decode Probabilities')
plt.title('0.1 W Decode Probabilities')
plt.xticks(index + bar_width, ('1000', '5000', '10000'))
plt.legend()

n_groups = 3
probabilities_standard = (0.99, 0.99, 0.99)
probabilities_enhanced1 = (0.99, 0.99, 0.99)
probabilities_enhanced2 = (0.99, 0.99, 0.99)
probabilities_enhanced3 = (0.99, 0.99, 0.99)

fig4, ax4 = plt.subplots()
index = np.arange(n_groups)
bar_width = 0.2
opacity = 1

rects14 = plt.bar(index, probabilities_standard, bar_width,
                  alpha=opacity,
                  color='Red',
                  label='Standard')

rects24 = plt.bar(index + bar_width, probabilities_enhanced1, bar_width,
                  alpha=opacity,
                  color='DarkOrange',
                  label='Enhanced 1')

rects34 = plt.bar(index + 2 * bar_width, probabilities_enhanced2, bar_width,
                  alpha=opacity,
                  color='Lime',
                  label='Enhanced 2')

rects44 = plt.bar(index + 3 * bar_width, probabilities_enhanced3, bar_width,
                  alpha=opacity,
                  color='DodgerBlue',
                  label='Enhanced 3')

plt.xlabel('Number of UAS')
plt.ylabel('Decode Probabilities')
plt.title('0.01 W Decode Probabilities')
plt.xticks(index + bar_width, ('1000', '5000', '10000'))
plt.legend()

plt.tight_layout()
plt.show()
