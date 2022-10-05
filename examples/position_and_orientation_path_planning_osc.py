"""
Similar to the position and orientation OSC example, with the addition of
an orientation path planner. Comparing to position_and_orientation_control_osc.py
you will see the smoothness of movement greatly improves by filtering the
target orientation in addition to the target position.

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

from abr_control.controllers import OSC, Damping
from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
from abr_control.utils import transformations
import abr_jaco2

plot = True
ctrlr_dof = [False, False, True, True, True, True]

# initialize our robot config
robot_config = abr_jaco2.Config()

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr = OSC(robot_config, kp=50, ko=100, kv=20, null_controllers=[damping],
            vmax=None, #vmax=[10, 10],  # [m/s, rad/s]
            # control (x, y, beta, gamma) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof=ctrlr_dof)

# create our interface
interface = abr_jaco2.Interface(robot_config)
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.START_ANGLES)

Pprof = Linear()
Vprof = Gaussian(dt=0.001, acceleration=1)
path = PathPlanner(pos_profile=Pprof, vel_profile=Vprof, verbose=True)

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

path.generate_path(
    start_position=hand_xyz,
    target_position=target_position,
    start_orientation=starting_orientation,
    target_orientation=target_orientation_euler,
    max_velocity=2,
    start_velocity=0,
    target_velocity=0,
)

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []
times = []

try:
    count = 0
    interface.init_force_mode()
    while count < 5000:
        start = timeit.default_timer()
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        # pos = target_position
        # orient = target_orientation_euler
        target = path.next()

        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=np.hstack((target[:3], target[6:9])),
            target_velocity=np.hstack((target[3:6], target[9:])),
            )

        # apply the control signal, step the sim forward
        interface.send_forces(np.array(u, dtype='float32'))

        # track data
        ee_track.append(np.copy(hand_xyz))
        hand_orient = transformations.euler_from_matrix(
            robot_config.R('EE', feedback['q']), axes='rxyz')
        ee_angles_track.append(np.copy(hand_orient))
        target_track.append(np.copy(target[:3]))
        target_angles_track.append(np.copy(target[6:9]))
        count += 1
        loop = timeit.default_timer() - start
        times.append(loop)

finally:
    # stop and reset the simulation
    interface.init_position_mode()
    interface.send_target_angles(robot_config.START_ANGLES)
    interface.disconnect()
    print('avg time: ', np.mean(times))
    print('max time: ', max(times))

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
