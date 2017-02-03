# Plots data from gravity compensation tests
# need to specify which test to plot in filename

import matplotlib.pyplot as plt
import numpy as np
import seaborn

filename = 'low_inertia_test1'

# Load Data
q_des = np.load('data/gravity_comp/%s/q_desired.npz' %
                filename)['q_desired']
# array format [num_joint, loop_count, num_position]
q = np.load('data/gravity_comp/%s/q_actual.npz' %
            filename)['q_actual'][:, 1:]

t = np.load('data/gravity_comp/%s/time.npz' %
            filename)['time'][1:, :]

# set plotting variables based on amount of runs and loop counts in saved data
num_joints = len(q)  # number of joint in arm
num_count = len(q[0])  # number of loop counts to stay in torque mode
num_positions = len(q[0][0])   # number of read positions
filename = 'baseline'
colours = np.array([['b--', 'g--', 'r--', 'k--', 'm--', 'c--'],
                    ['b', 'g', 'r', 'k', 'm', 'c']])

q_des = np.transpose(q_des)
error = np.sum((q_des[:, None, :] - q)**2, axis=1)

for hh in range(0, num_positions):
    plt.figure(figsize=(11,9))
    plt.subplot(211)
    for ii in range(0, num_joints):
        plt.plot(t[:, hh], q[ii, :, hh], colours[0, ii],
                 label='Joint %i' % ii)
        plt.plot(t[:, hh], q_des[ii, hh] * np.ones(num_count),
                 colours[1, ii])
        plt.xlabel('time[sec]')
        plt.ylabel('joint angle [rad]')
        plt.title('Joint angles for target position %i' % hh)
        plt.legend()#shadow=True, bbox_to_anchor=(1.02, 1), loc=2, 
            #borderaxespad=0.)        
    plt.subplot(212)
    for ii in range(0, num_joints):
        plt.plot(t[:, hh], q[ii, :, hh] - q_des[ii, hh],
                 colours[1, ii], label='%.3f' % error[ii, hh])
        plt.xlabel('time[sec]')
        plt.ylabel('q - q_des [rad]')
        plt.title('Joint angle errors for target position %i' % hh)
        plt.legend()#shadow=True, bbox_to_anchor=(1.02, 1), loc=2, 
            #borderaxespad=0.)
plt.tight_layout()
plt.show()