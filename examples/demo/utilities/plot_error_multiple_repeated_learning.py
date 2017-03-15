""" Plots the error for multiple trials of
the same run where learned weights are loaded
each time
"""

import numpy as np
import matplotlib.pyplot as plt
import seaborn
from mpl_toolkits.mplot3d import Axes3D

number_trials = 10
error_track = []

plt.figure()
plt.title('Mean error over trials')
plt.xlabel('Learning trial')
plt.ylabel('Mean error (m)')

for ii in range(0, number_trials+1):
    error = np.load(
        '../data/demo32/wrench/trial%i/error.npz' % ii)['arr_0']
    avg_error = np.mean(error)
    error_track.append(avg_error)
plt.plot(error_track)
plt.ylim([0, np.max(error_track)])
plt.show()
