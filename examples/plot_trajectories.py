import os
import numpy as np
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
import seaborn as sns

from abr_jaco2 import Config
rc = Config()

target = np.array([0.03, -.57, .87])
def bootstrapci(data, func, n=3000, p=0.95):
    index=int(n*(1-p)/2)
    samples = np.random.choice(data, size=(n, len(data)))
    r = [func(s) for s in samples]
    r.sort()
    return r[index], r[-index]

def get_data(folder, n_trials, run, sample_step):

    save_file = ('xyz_trials%i_run%i_samplestep%i' %
                 (n_trials, run, sample_step))
    saved_data = [sf for sf in os.listdir(folder)
                  if sf == (save_file + '.npz')]
    if len(saved_data) > 0:
        print('Loading data from ', saved_data[0])
        saved_data = np.load(folder + '/' + saved_data[0])
        raw_data_x = saved_data['x']
        raw_data_y = saved_data['y']
        raw_data_z = saved_data['z']
        print(raw_data_x.shape)
    else:
        raw_data_x = []
        raw_data_y = []
        raw_data_z = []
        for ii in range(n_trials):
            # get joint angles
            filename = (
                folder + '/trial%i/run%i_data/q%i.npz' % (ii, run, run))
            print(filename)

            # convert to xyz trajectories
            qs = np.load(filename)['q'].squeeze()[::sample_step]
            xyz = []
            for t in range(qs.shape[0]):
                xyz.append(rc.Tx('EE', q=qs[t], x=rc.OFFSET))
            xyz = np.array(xyz)

            raw_data_x.append(np.copy(xyz[:, 0]))
            print(xyz[:,0].shape)
            raw_data_y.append(np.copy(xyz[:, 1]))
            raw_data_z.append(np.copy(xyz[:, 2]))

        np.savez_compressed(
            folder + '/' + save_file,
            x=raw_data_x, y=raw_data_y, z=raw_data_z)

    return raw_data_x, raw_data_y, raw_data_z

alpha = .5
with sns.color_palette("OrRd", 8):
    fig = plt.figure(figsize=(9, 3))

    sample_step = 1
    # # get the data for no adaptation
    # x_non, y_non, z_non = get_data(
    #     'non_adaptive', n_trials=3, run=0, sample_step=sample_step)
    ax1 = plt.subplot(1, 3, 1)
    plt.xlabel('X')
    plt.ylabel('Y')
    # for x, y in zip(x_non, y_non):
    #     ax1.plot(x, y, 'ko', alpha=alpha)
    ax2 = plt.subplot(1, 3, 2)
    plt.xlabel('X')
    plt.ylabel('Z')
    # for x, z in zip(x_non, z_non):
    #     ax2.plot(x, z, 'ko', alpha=alpha)
    ax3 = plt.subplot(1, 3, 3)
    plt.xlabel('Y')
    plt.ylabel('Z')
    # for y, z in zip(y_non, z_non):
    #     ax3.plot(y, z, 'ko', alpha=alpha)
    # line_non, = plt.plot(100, 100, 'k')

    n_runs = 170
    n_trials = 1
    lines_spin = []
    for ii in range(n_runs):
        # # get the data for adaptation on cpu
        # mean_cpu, lower_bound_cpu, upper_bound_cpu = get_stats(
        #     'nengo_cpu', n_trials=10, n_runs=1, sample_step=sample_step)
        # for x, y in zip(x_non, y_non):
        #     plt.plot(x, y, 'C%io' % (ii*2+1), alpha=.01)
        # get the data for adaptation on spinnaker
        x_spin, y_spin, z_spin = get_data(
            'data', n_trials=n_trials, run=ii, sample_step=sample_step)

        color = 'C%io' % ((ii*2+1) % 10)
        for x, y in zip(x_spin, y_spin):
            ax1.plot(x, y, color, alpha=alpha)
        for x, z in zip(x_spin, z_spin):
            ax2.plot(x, z, color, alpha=alpha)
        for y, z in zip(y_spin, z_spin):
            ax3.plot(y, z, color, alpha=alpha)
        lines_spin.append(plt.plot(100, 100, color)[0])
        ax1.plot(target[0], target[1], 'bo', mew=2)
        ax2.plot(target[0], target[2], 'bo', mew=2)
        ax3.plot(target[1], target[2], 'bo', mew=2)
        print('TARGET: ', target)

    xlim = [-.25, .75]
    ylim = [-.5, .5]
    zlim = [.3, 1.3]
    ax1.set_aspect('equal')
    ax1.set_xlim(xlim)
    ax1.set_ylim(ylim)

    ax2.set_aspect('equal')
    ax2.set_xlim(xlim)
    ax2.set_ylim(zlim)

    ax3.set_aspect('equal')
    ax3.set_xlim(ylim)
    ax3.set_ylim(zlim)

    # plt.legend([line_non] + lines_spin,
            # ['No adaptation'] +
    # plt.legend(lines_spin,\
    #         ['Nengo SpiNNaker trial%i' % ii for ii in range(n_runs)])\

    plt.tight_layout()
    plt.show()
	
