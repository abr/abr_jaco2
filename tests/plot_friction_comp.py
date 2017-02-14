import numpy as np
import matplotlib.pyplot as plt
import seaborn

error = np.load('error.npz')['error']
kp = np.load('kp.npz')['kp']
kv = np.load('kv.npz')['kv']
target_pos = np.load('target_pos.npz')['target_pos']
joint_angles = np.load('joint_angles.npz')['joint_angles']
torques_sent = np.load('torques_sent.npz')['torques_sent']
torques_read = np.load('torques_read.npz')['torques_read']
friction = np.load('friction.npz')['friction']
times = np.load('times.npz')['times']

F_min = np.array([1.75, 0.20, 1.17, 0.92, 0.90, 0.84])

plt.figure()
for ii in range(0, 6):
    plt.subplot(4, 3, (ii + 1))
    plt.title('Joint %i Angles, Kp = %f Kv = %f' % (ii, kp, kv))
    plt.xlabel('loop count')
    plt.ylabel('joint angle (rad)')
    plt.plot((joint_angles[ii, :] + np.pi) % (np.pi * 2) - np.pi, label='Current')
    plt.plot(
        ((np.ones(joint_angles[ii, :].shape) * target_pos[ii])
         + np.pi) % (np.pi * 2) - np.pi,
        '--', label='Target')
    plt.legend(range(6))

    plt.subplot(4, 3, (ii + 1) + 6)
    plt.title('Joint %i Torques, Total Error: %f' % (ii, error))
    plt.xlabel('loop count')
    plt.ylabel('torque (Nm)')
    plt.plot(friction[ii, :], 'r', label='Friction Compensation')
    plt.plot(-1.0 * torques_read[ii, :], label='Read')
    plt.plot(torques_sent[ii, :], '--', label='Sent')    

    F_min_pos = F_min[ii] * np.ones(times.shape)
    F_min_neg = -1.0 * F_min_pos
    plt.fill_between(range(joint_angles[ii].shape[0]),
                     F_min_neg, F_min_pos, alpha=.25, facecolor='yellow',label='Friction Threshold')
    plt.legend()

plt.show()
