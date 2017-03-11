""" Plots the error for multiple trials of
the same run where learned weights are loaded
each time
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

number_trials = 35
number_targets = 3
error_track = []

plt.figure()
plt.title('Error')
plt.xlabel('Loop Count')
plt.ylabel('Error')

for ii in range(0, number_trials+1):
    error = np.load('../data/error%i.npz' % ii)['arr_0']
    avg_error = np.mean(error)
    error_track.append(avg_error)
plt.plot(error_track)
plt.show()
