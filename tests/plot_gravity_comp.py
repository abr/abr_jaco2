# Plots data from gravity compensation tests
# need to specify which test to plot in filename

import matplotlib.pyplot as plt
import numpy as np
import seaborn

filename = 'baseline'

# Load Data
q_des = np.load('data/gravity_comp/%s/q_desired.npz' %
                filename)['q_desired']
# array format [num_joint, loop_count, num_position]
q = np.load('data/gravity_comp/%s/q_actual.npz' %
            filename)['q_actual'][:, 1:]

# set plotting variables based on amount of runs and loop counts in saved data
num_joints = len(q)  # number of joint in arm
num_count = len(q[0])  # number of loop counts to stay in torque mode
num_positions = len(q[0][0])   # number of read positions
filename = 'baseline'
x = np.linspace(0.0, 1.0, num_count - 1)
colours = np.array([['bo', 'go', 'ro', 'ko', 'mo', 'co'],
                    ['b', 'g', 'r', 'k', 'm', 'c']])

q_des = np.transpose(q_des)
error = np.sum((q_des[:, None, :] - q)**2, axis=1)

for hh in range(0, num_positions):
    plt.figure()
    plt.subplot(211)
    for ii in range(0, num_joints):
        plt.plot(x, q[ii, 1:, hh], colours[0, ii],
                 label='Joint %i Actual' % ii)
        plt.plot(x, q_des[ii, hh] * np.ones(num_count - 1),
                 colours[1, ii])
        plt.xlabel('norm(loop count)')
        plt.ylabel('joint angle [rad]')
        plt.title('Joint angles for target position %i' % hh)
        plt.legend(shadow=True)
    plt.subplot(212)
    for ii in range(0, num_joints):
        plt.plot(x, q[ii, 1:, hh] - q_des[ii, hh],
                 colours[1, ii], label='%.3f' % error[ii, hh])
        plt.xlabel('norm(loop count)')
        plt.ylabel('q - q_des [rad]')
        plt.title('Joint angle errors for target position %i' % hh)
        plt.legend(shadow=True)
        # total_error_over_time = np.linalg.norm(
        #     q_des[hh,ii] - q[ii, :, hh], axis=1)
        # total_error = np.sum(total_error_over_time)
plt.show()
"""error = np.sum((q_des[:, None, :] - q)**2, axis=1)
print("error: ", error)
print('x ', len(error))
print('y ', len(error[0]))
print('z ', len(error[0][0]))"""
# calculate the error
# error_of each joint = q_des - q
# plot this over time, separate plot for each different target position
# total_error_over_time = np.linalg.norm(q_des - q, axis=1)
# total_error = np.sum(total_error_over_time)
