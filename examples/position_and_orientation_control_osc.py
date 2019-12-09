"""
Running operational space control. The controller will
move the end-effector to the target object's position and orientation.
The orientation along with z cartesian target location will
be maintained. The x-y directions are not being controlled so you should
be able to move the arm along the xy plane freely while the controller
attempts to maintain (z, a, b, g)
"""
import numpy as np

import abr_jaco2
from abr_control.controllers import OSC, Damping, path_planners
from abr_control.utils import transformations


# initialize our robot config
robot_config = abr_jaco2.Config()

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr_dof = [False, False, True, True, True, True]
dof_labels = np.array(['x', 'y', 'z', 'alpha', 'beta', 'gamma'])
print('Controlling %s dof, %s are free to be manipulated'
       % (dof_labels[ctrlr_dof], dof_labels[[not i for i in ctrlr_dof]]))

ctrlr = OSC(robot_config, kp=50, kv=20, ko=150, null_controllers=[damping],
            # vmax=[1, 1],  # [m/s, rad/s]
            # control (x, y, beta, gamma) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof=ctrlr_dof,
            orientation_algorithm=0)


# create our interface
interface = abr_jaco2.Interface(robot_config)

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []


path = path_planners.SecondOrderFilter(
    n_timesteps=3000,
    w=1e4, zeta=3, threshold=0.1, dt=0.003)

interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.START_ANGLES)
interface.init_force_mode()

feedback = interface.get_feedback()
target_angles = list(transformations.euler_from_matrix(
    robot_config.R('EE', feedback['q']), axes='sxyz'))
target_angles[2] += 1.5
target_angles[1] += 1.5

target_vel = np.zeros(3)
target_xyz = np.array([0, 0.4, 0.60])
shift = 0.30
target_pos = robot_config.Tx('EE', feedback['q'])

try:
    count = 0
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        target_pos, target_vel = path._step(
            position=target_pos, velocity=target_vel, target_pos=target_xyz)

        target = np.hstack((target_pos, target_angles))

        if count %3000 == 0:
            target_xyz[2] = target_xyz[2] + -1*shift
            shift *= -1
            # ii = 3
            # target_angles[-ii] = (target_angles[-ii] + 0.2) % (2*np.pi)

        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            target_vel=np.hstack((target_vel, np.zeros(3)))
            )

        if count % 1000 == 0:
            print(u)
        # apply the control signal, step the sim forward
        interface.send_forces(np.array(u, dtype='float32'))

        # track data
        ee_track.append(np.copy(hand_xyz))
        ee_angles_track.append(transformations.euler_from_matrix(
            robot_config.R('EE', feedback['q'])))
        target_track.append(np.copy(target[:3]))
        target_angles_track.append(np.copy(target[3:]))
        count += 1

finally:
    # stop and reset the simulation
    interface.init_position_mode()
    interface.send_target_angles(robot_config.START_ANGLES)
    interface.disconnect()
    print('Disconnected')

    ee_track = np.array(ee_track)
    ee_angles_track = np.array(ee_angles_track)
    target_track = np.array(target_track)
    target_angles_track = np.array(target_angles_track)

    ls = ['-', '--']
    cols = ['r', 'b', 'g']
    labelpos = ['x', 'y', 'z']
    labelrot = ['a', 'b', 'g']
    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(12,8))
        ax1 = fig.add_subplot(311)
        ax1.set_ylabel('3D position (m)')
        for cnt, ee in enumerate(ee_track.T):
            if ctrlr_dof[cnt]:
                ax1.plot(ee, c=cols[cnt], linestyle=ls[0],
                        label='EE: %s' % (labelpos[cnt]))
        for cnt, tt in enumerate(target_track.T):
            if ctrlr_dof[cnt]:
                ax1.plot(tt, c=cols[cnt], linestyle=ls[1],
                        label='T: %s' % (labelpos[cnt]))
        ax1.legend()

        ax2 = fig.add_subplot(312)
        for cnt, ee in enumerate(ee_angles_track.T):
            if ctrlr_dof[cnt+3]:
                ax2.plot(ee, c=cols[cnt], linestyle=ls[0],
                        label='EE: %s' % (labelrot[cnt]))
                ax2.plot(target_angles_track.T[cnt], c=cols[cnt], linestyle=ls[1],
                        label='EE T: %s' % (labelrot[cnt]))
        ax2.legend()

        ax2.set_ylabel('3D orientation (rad)')
        ax2.set_xlabel('Time (s)')

        ax3 = fig.add_subplot(313, projection='3d')
        ax3.set_title('End-Effector Trajectory')
        ax3.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax3.scatter(target_track[0, 0], target_track[0, 1], target_track[0, 2],
                    label='target', c='g')
        ax3.legend()

        plt.tight_layout()
        plt.show()
