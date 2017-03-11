""" Plots the error from one run of learning that loops
through the same number of targets (number_targets)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import abr_jaco2

error = np.load('../data/error0.npz')['arr_0']
targets = np.load('../data/target0.npz')['arr_0']
previous_run = 0
target_number = 1
number_targets = 3
trial_counter = 0
error_track = []
plt.figure()
plt.title('Average Error: %f' % np.mean(error))
plt.xlabel('Loop Count')
plt.ylabel('Error')

for ii in range(1,len(targets)):
    if targets[ii,0] != targets[ii-1,0]:
        target_number += 1
        if target_number%number_targets == 0:
            avg_error = np.mean(error[previous_run/100:(ii/100)+1])
            error_track.append(avg_error)
            trial_counter += 1
            print('prev run: ', previous_run)
            print('ii+1: ', ii+1)
            previous_run = ii
            print('avg_error for run %i: %f' % (trial_counter-1, avg_error))
            print('error: ', error[previous_run:ii+1])
plt.plot(error_track)
plt.show()
print(error_track)
