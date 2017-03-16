""" Plots the error from one run of learning that loops
through the same number of targets (number_targets)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import abr_jaco2

data_folder = ('../data/deliverable5/tool_data/heatgun/' +
    'filter_0.003/null_off/')
adaptive = data_folder + 'adaptive/5e-5/'
non_adapt_low_gain = data_folder + 'non-adaptive/low_gain/'
non_adapt_high_gain = data_folder + 'non-adaptive/high_gain/'
num_trials = 9
error_track = []
time_track = []
plt.figure()

for ii in range(0,num_trials+1):
    error = np.load('%s/trial%i/error.npz' %
        (adaptive, ii))['arr_0']
    error_track.append(np.copy(np.sum(error)))

    time = np.load('%s/trial%i/time.npz' %
        (adaptive, ii))['arr_0']
    time_track.append(np.copy(np.sum(time)))

plt.subplot(2,1,1)
plt.title('Total Error Over Reach')
plt.xlabel('Trial Number')
plt.ylabel('Total Error [cm]')
plt.plot(error_track, label='Adaptive')
plt.plot(np.arange(num_trials+1), np.ones(num_trials+1) * np.sum(np.load(non_adapt_low_gain+'error.npz')['arr_0']),
         'r--', label='Non-adaptive')
plt.plot(np.arange(num_trials+1), np.ones(num_trials+1) * np.sum(np.load(non_adapt_high_gain+'error.npz')['arr_0']),
         'g-.', label='Non-adaptive High Gain')
plt.legend()

plt.subplot(2,1,2)
plt.title('Time to Target')
plt.xlabel('Trial Number')
plt.ylabel('Time [sec]')
plt.plot(time_track, label='Adaptive')
plt.plot(np.arange(num_trials+1), np.ones(num_trials+1) * 60,
         'r--', label='Non-adaptive')
plt.plot(np.arange(num_trials+1), np.ones(num_trials+1) * 60,
         'g-.', label='Non-adaptive High Gain')
plt.legend()
plt.show()
#print(list.range(0, len(error_track)), error_track)
