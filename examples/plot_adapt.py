import numpy as np
import os
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt

folder = ('~/.cache/abr_control/saved_weights/on_the_fly/'
          + 'nengo_spinnaker/trial0/')
folder = os.path.expanduser(folder)
time = []
q = []
u = []
adapt = []
target = []
num_runs = 1
ii=0
# for ii in range(0, num_runs):
q = np.squeeze(np.load(folder + 'run%i_data/q%i.npz'%(ii, ii))['q']).T
print('q',q)
u = np.squeeze(np.load(folder + 'run%i_data/u%i.npz'%(ii, ii))['u']).T
print('u',u)
adapt = np.squeeze(np.load(folder + 'run%i_data/adapt%i.npz'%(ii, ii))['adapt']).T
print('adapt',adapt)
time = (np.load(folder + 'run%i_data/time%i.npz'%(ii, ii))['time']).flatten()
print('time',time)
target = np.squeeze(np.load(folder + 'run%i_data/target%i.npz'%(ii, ii))['target']).T
    # q.append(np.copy(np.load(folder + 'q%i'%ii)['q']))
    # u.append(np.copy(np.load(folder + 'u%i'%ii)['u']))
    # adapt.append(np.copy(np.load(folder + 'adapt%i'%ii)['adapt']))
    # time.append(np.copy(np.load(folder + 'time%i'%ii)['time']))
    # target.append(np.copy(np.load(folder + 'target%i'%ii)['target']))
print(np.array(time).shape)
print(np.array(q[1,None]).shape)
print(np.array(u).shape)
print(np.array(adapt).shape)
plt.figure()
# plt.subplot(2,2,1)
plt.title('q')
for ii in range(0,6):
    # plt.plot(time, q[ii])
    plt.plot(np.cumsum(time), q[ii])
# plt.subplot(2,2,2)
# plt.title('u')
# for ii in range(0,6):
#     plt.plot(time, u[ii, None])
# plt.subplot(2,2,3)
# plt.title('adapt')
# for ii in range(0,6):
#     plt.plot(time, adapt[ii, None])
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