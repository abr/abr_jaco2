"""
Plots error vs time for 10 averaged on the fly learning, comapring nengo,
nengo_spinnaker, and no adaptation. Data in 'on_the_fly' folder
"""

import numpy as np
import os
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt

folder = ('~/.cache/abr_control/saved_weights/repeated/')
folder = os.path.expanduser(folder)

num_trials = 100
t_num =0
plt_step = 10
time = []
q = []
u = []
adapt = []
target = []
error = []
error_avg = []
training_track = []
labels = ['nengo']#, 'nengo_cpu', 'non_adaptive']

plt.figure()
# plt.subplot(2,2,1)
plt.title('Error')
for nn in range(0, len(labels)):
    for ii in range(0, num_trials):
        # q[ii] = np.squeeze(np.load(folder + labels[nn] + '/trial%i/run0_data/q0.npz'%(ii))['q'])
        u.append(np.squeeze(np.load(folder + labels[nn]
            + '/trial%i/run%i_data/u%i.npz'%(t_num,ii,ii))['u']))
        # adapt[ii] = np.squeeze(np.load(folder + labels[nn]
        #                    + '/trial%i/run0_data/adapt0.npz'%(ii))['adapt'])
        # time.append((np.load(folder + labels[nn]
        #     + '/trial0/runii_data/timeii.npz'%(ii,ii))['time']).flatten())
        error.append((np.load(folder + labels[nn]
                     + '/trial%i/run%i_data/error%i.npz'%(t_num,ii,ii))['error']).flatten())
        # training_track[ii] = np.squeeze((np.load(folder + labels[nn]
        #          + '/trial%i/run0_data/training0.npz'%(ii))['training']))
        # target = np.squeeze(np.load(folder + labels[nn]
        #                     + '/trial%i/run0_data/target0.npz'%(ii, ii))['target']).T
        error_avg.append(np.average(np.array(error[ii])))

    # plt.plot(np.cumsum((time)),
    #     (error), label = labels[nn])
    x = range(num_trials)[::plt_step]
    plt.xticks(x, x)
    plt.plot(error_avg)
    # plt.plot(u[4])
    # plt.legend(range(0,6))
# plt.plot(error)
# plt.plot(time)
# plt.subplot(2,2,2)
# plt.title('u')
# for ii in range(0,6):
#     plt.plot(time, u[ii, None])
# plt.subplot(2,2,3)
# plt.title('adapt')
# for ii in range(0,6):
#     plt.plot(time, adapt[ii, None])
plt.legend()
plt.show()
# print('TARGET: ', target)
    # ee_track = np.array(ee_track)
    # target_track = np.array(target_track)
    #
    # if ee_track.shape[0] > 0:
    #     # plot distance from target and 3D trajectory
    #     import matplotlib
    #     matplotlib.use("TKAgg")
    #     import matplotlib.pyplot as plt
    #     from abr_control.utils.plotting import plot_3D
    #
    #     plt.figure()
    #     plt.plot(np.sqrt(np.sum((np.array(target_track) -
    #                              np.array(ee_track))**2, axis=1)))
    #     plt.ylabel('Distance (m)')
    #     plt.xlabel('Time (ms)')
    #     plt.title('Distance to target')
    #
    #     plot_3D(ee_track, target_track)
    #     plt.show()
