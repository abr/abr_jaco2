""" Plots the error from one run of learning that loops
through the same number of targets (number_targets)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import abr_jaco2

data_folder = '../data/deliverable5/wrench/'
baseline = ('../data/deliverable5/wrench/60seconds' +
            '/filter_0.003/compliant_baseline')
num_trials = 10
error_track = []
time_track = []
plt.figure()

for ii in range(0,num_trials+1):
    error = np.load('%s/trial%i/error.npz' %
        (data_folder, ii))['arr_0']
    error_track.append(np.copy(np.sum(error)))

    time = np.load('%s/trial%i/time.npz' %
        (data_folder, ii))['arr_0']
    time_track.append(np.copy(np.sum(time)))

plt.subplot(2,1,1)
plt.title('Total Error Over Reach')
plt.xlabel('Trial Number')
plt.ylabel('Total Error [cm]')
plt.plot(error_track, label='Adaptive')
plt.plot(np.arange(11), np.ones(11) * np.sum(np.load(baseline+'/error.npz')['arr_0']),
         'r--', label='Non-adaptive')
plt.legend()

plt.subplot(2,1,2)
plt.title('Time to Target')
plt.xlabel('Trial Number')
plt.ylabel('Time [sec]')
plt.plot(time_track, label='Adaptive')
plt.plot(np.arange(11), np.ones(11) *60,
         'r--', label='Non-adaptive')
plt.legend()
plt.show()
#print(list.range(0, len(error_track)), error_track)
