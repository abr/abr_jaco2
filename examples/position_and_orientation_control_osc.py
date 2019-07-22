"""
Running operational space control. The controller will
move the end-effector to the target object's position and orientation.
The orientation along with the y and z cartesian target locations will
be maintained. The x direction is not being controlled so you should
be able to move the arm along the x axis freely while the controller
attempts to maintain (y, z, a, b, g)
"""
import numpy as np

import abr_jaco2
from abr_control.controllers import OSC, Damping, path_planners
from abr_control.utils import transformations


# initialize our robot config
robot_config = abr_jaco2.Config(use_cython=True, hand_attached=True)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr = OSC(robot_config, kp=50, kv=20, ko=150, null_controllers=[damping],
            # vmax=[1, 1],  # [m/s, rad/s]
            # control (x, y, beta, gamma) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof = [False, True, True, True, True, True],
            orientation_algorithm=0)


# create our interface
interface = abr_jaco2.Interface(robot_config)

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []
dr_track = []
dr_err = []


path = path_planners.SecondOrder(
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
    import redis
    r = redis.StrictRedis('localhost')
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

        r.set(
            'norm_target_xyz_robot_coords', '%.3f %.3f %.3f'
            % (hand_xyz[0], target[1], target[2]))
        r.set('q', '%.3f %.3f %.3f %.3f %.3f %.3f' %
                             (feedback['q'][0],feedback['q'][1],feedback['q'][2],
                              feedback['q'][3],feedback['q'][4],feedback['q'][5]))
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
        # dr_track.append(np.copy(ctrlr.dr))
        # dr_err.append(np.copy(np.linalg.norm(ctrlr.dr, 2)))
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
    dr_track = np.array(dr_track)
    dr_err = np.array(dr_err)

    ls = ['-', '--']
    cols = ['r', 'b', 'g']
    labelpos = ['x', 'y', 'z']
    labelrot = ['a', 'b', 'g']
    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(12,8))
        ax1 = fig.add_subplot(221)
        ax1.set_ylabel('3D position (m)')
        for cnt, ee in enumerate(ee_track.T):
            ax1.plot(ee, c=cols[cnt], linestyle=ls[0],
                    label='EE: %s' % (labelpos[cnt]))
        for cnt, tt in enumerate(target_track.T):
            ax1.plot(tt, c=cols[cnt], linestyle=ls[1],
                    label='T: %s' % (labelpos[cnt]))
        ax1.legend()

        ax2 = fig.add_subplot(222)
        for cnt, ee in enumerate(ee_angles_track.T):
            ax2.plot(ee, c=cols[cnt], linestyle=ls[0],
                    label='EE: %s' % (labelrot[cnt]))
        #for cnt, tat in enumerate(target_angles_track.T):
        for cnt, tat in enumerate(dr_track.T):
            ax2.plot(tat, c=cols[cnt], linestyle=ls[1],
                    label='T: %s' % (labelrot[cnt]))
        ax2.legend()

        ax2.set_ylabel('3D orientation (rad)')
        ax2.set_xlabel('Time (s)')

        ax3 = fig.add_subplot(223, projection='3d')
        ax3.set_title('End-Effector Trajectory')
        ax3.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax3.scatter(target_track[0, 0], target_track[0, 1], target_track[0, 2],
                    label='target', c='g')
        ax3.legend()

        ax4 = fig.add_subplot(224)
        ax4.set_title('Norm dr')
        ax4.plot(dr_err)

        plt.tight_layout()
        plt.show()
