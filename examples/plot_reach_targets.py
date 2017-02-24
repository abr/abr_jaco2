import numpy as np
import matplotlib.pyplot as plt
import seaborn
import abr_control

error = np.load('error.npz')['error']
kp = np.load('kp.npz')['kp']
kv = np.load('kv.npz')['kv']
target_pos = np.load('target_pos.npz')['target_pos']
joint_angles = np.load('joint_angles.npz')['joint_angles']
torques_sent = np.load('torques_sent.npz')['torques_sent']
torques_read = np.load('torques_read.npz')['torques_read']
friction = np.load('friction.npz')['friction']
velocities = np.load('velocity.npz')['velocity']
times = np.load('times.npz')['times']
F_brk = np.load('F_brk.npz')['F_brk']
ee_track = np.load('ee_track.npz')['ee_track']
target_track = np.load('target_track.npz')['target_track']
u_track = np.load('u_track.npz')['u_track']
#C = np.load('C.npz')['C']
#g = np.load('g.npz')['g']
#training = np.load('training.npz')['training']
#x_tilde = np.load('x_tilde.npz')['x_tilde']


plt.figure()
for ii in range(0, 6):
    plt.subplot(4, 3, (ii + 1))
    plt.title('Joint %i Angles, Kp = %f Kv = %f' % (ii, kp, kv))
    plt.xlabel('loop count')
    plt.ylabel('joint angle (rad)')
    plt.plot((joint_angles[ii, :] + np.pi) % (np.pi * 2) - np.pi, label='Current Angle')
    """plt.plot(
        ((np.ones(joint_angles[ii, :].shape) * target_pos[ii])
         + np.pi) % (np.pi * 2) - np.pi,
        '--', label='Target Angle')"""
    plt.plot(velocities[ii, :], '--m', label='Joint Velocity')
    plt.legend()

    plt.subplot(4, 3, (ii + 1) + 6)
    plt.title('Joint %i Torques, Total Error: %f' % (ii, error))
    plt.xlabel('loop count')
    plt.ylabel('torque (Nm)')
    plt.plot(friction[ii, :], 'r', label='Friction Compensation') 
    plt.plot(torques_sent[ii, :] - friction[ii, :], label='Control Signal')
    plt.plot(torques_sent[ii, :], 'b--', label='Total Torque') 
    #plt.plot(-1.0*C[ii, :], 'r', label='Coriolis')
    #plt.plot(-1.0*g[ii, :], 'g', label='Gravity')
    #plt.plot(training[ii, :], 'k', label='training')

    #plt.plot(velocities[ii, :], '--m', label='Joint Velocity')
    #plt.plot(velocities[ii, :], '--m', label='Joint Velocity')  
     

    F_brk_pos = F_brk[ii] * np.ones(times.shape)
    F_brk_neg = -1.0 * F_brk_pos
    plt.fill_between(range(joint_angles[ii].shape[0]),
                     F_brk_neg, F_brk_pos, alpha=.25, facecolor='yellow',label='Friction Threshold')
    plt.legend()

# plot.show() in util plotting script
"""plt.figure()
for ii in range(0,3):
    plt.subplot(3,1, (ii+1))
    plt.title('x_tilde')
    plt.plot(x_tilde[ii,:])
    plt.legend(range(0,3))"""

    #for jj in range(0,len(x_tilde[0])):
    #    if x_tilde[ii,jj] < 0.05:
    #        plt.axvline(x_tilde[ii,jj])
abr_control.utils.plotting.plot_trajectory(ee_track, target_track)