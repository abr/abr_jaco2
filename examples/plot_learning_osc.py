# Plots trajectory, error, and time to target of deliverable 4.3

import matplotlib.pyplot as plt
import numpy as np
import os

if os.path.getsize('data/learning_osc/read_info.txt') <= 0:
    #folder = 'INPUT/DEFAULT/LOCATION'
    print('file location missing')
else:
    with open('data/learning_osc/read_info.txt') as myfile:
        file_location = (myfile.readline()).split(',')

    test_name = file_location[0]
    n_neurons = int(file_location[1])
    num_trials = int(file_location[2])
    num_runs = int(file_location[3])

folder = 'data/learning_osc/%s/%i_neurons' % (test_name, n_neurons)

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(1, 1, 1, projection='3d')
# baseline is the non-adaptive case
ee_baseline = np.load('data/learning_osc/baseline/ee_no_learn.npz')['ee']
target = np.load('data/learning_osc/baseline/targets.npz')['targets'][0]
ax.plot(ee_baseline[:, 0], ee_baseline[:, 1],
        ee_baseline[:, 2], 'k', lw=2)

times = np.zeros((num_trials, num_runs))
avg_times = np.zeros(num_runs)
error_sums = np.zeros((num_trials, num_runs))
trial_lengths = np.zeros((num_trials, num_runs))

for ii in range(num_trials):
    for jj in range(num_runs):
        ee = np.load('%s/trial%i/ee%i.npz' % (folder, ii, jj))['ee']

        # load times to target
        with open('%s/trial%i/total_time_track.txt' % (folder, ii)) as f:
            times_buffer = f.read().split()
            times[ii] = [float(i) for i in times_buffer]

        if ii == num_trials - 1 and jj == num_runs - 1:
            plt.figure(1)
            # plot start point of hand
            ax.plot([ee[0, 0]], [ee[0, 1]],
                    [ee[0, 2]], 'bx', mew=10)
            # plot trajectory of hand
            ax.plot(ee[:, 0], ee[:, 1],
                    ee[:, 2])
            # plot trajectory of target
            ax.plot([target[0]], [target[1]],
                    [target[2]], 'rx', mew=10)

        error_sums[ii, jj] = np.sum(
            np.sqrt(np.sum((ee - target)**2, axis=1))) * .003
        trial_lengths[ii, jj] = ee.shape[0]

avg_error = np.sum(error_sums, axis=0) / float(num_trials)
avg_length = np.sum(trial_lengths, axis=0) / float(num_trials)

# calc avg times
for nn in range(num_runs):
    avg_times[nn] = sum(times[:, nn])/float(num_trials)

# plot total error
plt.figure()
plt.subplot(2, 1, 1)
plt.fill_between(range(avg_error.shape[0]),
                 np.min(error_sums, axis=0),
                 np.max(error_sums, axis=0),
                 facecolor='blue',
                 alpha=.25,
                 interpolate=True)
plt.plot(avg_error, lw=3)
plt.ylabel('Total error over movement (m)')
plt.subplot(2, 1, 2)

plt.fill_between(range(avg_times.shape[0]),
                 np.min(times, axis=0),
                 np.max(times, axis=0),
                 facecolor='blue',
                 alpha=.25,
                 interpolate=True)
plt.plot(avg_times, lw=3)
plt.ylabel('Length of movement (s)')

plt.xlabel('Trials')
plt.savefig('%s/results' % folder)

plt.show()
