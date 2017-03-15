""" Plots the error from one run of learning that loops
through the same number of targets (number_targets)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import abr_jaco2

data_folder = 'data/deliverable5/wrench'
num_trials = 3
error_track = []
plt.figure()
plt.title('Total Error Over Reach')
plt.xlabel('Trial Number')
plt.ylabel('Total Error [cm]')

for ii in range(0,num_trials):
    error = np.load('../%s/trial%i/error.npz' %
        (data_folder, ii))['arr_0']
    error_track.append(np.copy(np.sum(error)))
plt.plot(error_track)
plt.show()
print(list.range(0, len(error_track)), error_track)
