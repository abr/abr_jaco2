"""
Running operational space control using VREP. The controller will
move the end-effector to the target object's position and orientation.

This example controls 4 degrees of freedom (1 position and 3 orientation),
and applies a second order path planner to both position and orientation targets

After termination the script will plot results

Only the z cardinal direction is maintained by the controller, you should be
able to move the arm around in the xy plane.

The target orientation is randomly generated at run time
"""
import numpy as np
import timeit

from abr_control.controllers import OSC, Damping, path_planners
from abr_control.utils import transformations
import abr_jaco2

plot = False
ctrlr_dof = [False, False, True, True, True, True]

# initialize our robot config
robot_config = abr_jaco2.Config(use_cython=True, hand_attached=True)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr = OSC(robot_config, kp=30, ko=100, kv=20, null_controllers=[damping],
            vmax=None, #vmax=[10, 10],  # [m/s, rad/s]
            # control (x, y, beta, gamma) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof=ctrlr_dof)


# create our interface
interface = abr_jaco2.Interface(robot_config)
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.START_ANGLES)

# pregenerate our path and orientation planners
n_timesteps = 10000
# traj_planner = path_planners.BellShaped(
#     error_scale=1, n_timesteps=n_timesteps)
traj_planner = path_planners.SecondOrder(n_timesteps=n_timesteps, dt=0.004)

feedback = interface.get_feedback()
hand_xyz = robot_config.Tx('EE', feedback['q'])
starting_orientation = robot_config.quaternion('EE', feedback['q'])

target_orientation = np.random.random(3)
target_orientation /= np.linalg.norm(target_orientation)
# convert our orientation to a quaternion
target_orientation = [0] + list(target_orientation)
print(target_orientation)
target_orientation_euler = transformations.euler_from_quaternion(target_orientation)
target_position = np.array([-0.4, -0.3, 0.5]) #np.random.random(3)

traj_planner.generate_path(position=hand_xyz, target_pos=target_position,
                           velocity=np.zeros(3))
_, orientation_planner = traj_planner.generate_orientation_path(
    orientation=starting_orientation, target_orientation=target_orientation)

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []
times = []

try:
    count = 0
    interface.init_force_mode()
    #while count < 5000:
    while 1:
        start = timeit.default_timer()
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        # pos = target_position
        # orient = target_orientation_euler
        pos, vel = traj_planner.next()
        orient = orientation_planner.next()
        target = np.hstack([pos, orient])

        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            #target_vel=np.hstack([vel, np.zeros(3)])
            )

        # apply the control signal, step the sim forward
        interface.send_forces(np.array(u, dtype='float32'))

        # track data
        ee_track.append(np.copy(hand_xyz))
        hand_orient = transformations.euler_from_matrix(
            robot_config.R('EE', feedback['q']), axes='rxyz')
        ee_angles_track.append(np.copy(hand_orient))
        target_track.append(np.copy(target[:3]))
        target_angles_track.append(np.copy(target[3:]))
        count += 1
        loop = timeit.default_timer() - start
        times.append(loop)

finally:
    # stop and reset the simulation
    print(np.array(ee_track).shape)
    interface.init_position_mode()
    interface.send_target_angles(robot_config.START_ANGLES)
    interface.disconnect()
    print('avg time: ', np.mean(times))
    print('max time: ', max(times))

    ee_track = np.array(ee_track).T
    ee_angles_track = np.array(ee_angles_track).T
    target_track = np.array(target_track).T
    target_angles_track = np.array(target_angles_track).T

    if plot and ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611
        label_pos = ['x', 'y', 'z']
        label_or = ['a', 'b', 'g']
        c = ['r', 'g', 'b']

        fig = plt.figure(figsize=(8,12))
        ax1 = fig.add_subplot(311)
        ax1.set_ylabel('3D position (m)')
        for ii, ee in enumerate(ee_track):
            ax1.plot(ee, label='EE: %s' % label_pos[ii], c=c[ii])
            ax1.plot(target_track[ii], label='Target: %s' % label_pos[ii],
                     c=c[ii], linestyle='--')
        ax1.legend()

        ax2 = fig.add_subplot(312)
        for ii, ee in enumerate(ee_angles_track):
            ax2.plot(ee, label='EE: %s' % label_or[ii], c=c[ii])
            ax2.plot(target_angles_track[ii], label='Target: %s'%label_or[ii],
                     c=c[ii], linestyle='--')
        ax2.set_ylabel('3D orientation (rad)')
        ax2.set_xlabel('Time (s)')
        ax2.legend()

        ee_track = ee_track.T
        target_track = target_track.T
        ax3 = fig.add_subplot(313, projection='3d')
        ax3.set_title('End-Effector Trajectory')
        ax3.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax3.plot(target_track[:, 0], target_track[:, 1], target_track[:, 2], label='ee_xyz', c='g', linestyle='--')
        ax3.scatter(target_track[-1, 0], target_track[-1, 1], target_track[-1, 2],
                    label='target', c='g')
        ax3.legend()
        plt.show()
