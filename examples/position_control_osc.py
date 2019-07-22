"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1500 time steps, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import traceback

import abr_jaco2
# from abr_control.arms import jaco2 as arm
# from abr_control.arms import onelink as arm
from abr_control.controllers import OSC, Damping
from abr_control.utils import transformations

# initialize our robot config
robot_config = abr_jaco2.Config(use_cython=True, hand_attached=True)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# instantiate controller
ctrlr = OSC(robot_config, kp=20, null_controllers=[damping],
            vmax=[0.5, 0],  # [m/s, rad/s]
            # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof = [True, True, True, False, False, False])


# create our VREP interface
interface = abr_jaco2.Interface(robot_config)
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.START_ANGLES)

# set up lists for tracking data
ee_track = []
target_track = []

target = np.array([.56, -.09, .52, 0, 0, 0])

try:
    # get the end-effector's initial position
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', feedback['q'])

    # make the target offset from that start position
    target_xyz = start + np.array([0.2, -0.2, -0.3])

    count = 0.0
    interface.init_force_mode()
    while count < 1500:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            )

        # send forces into VREP, step the sim forward
        interface.send_forces(np.array(u, dtype='float32'))

        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target[:3]))

        count += 1

except:
    print(traceback.format_exc())

finally:
    # stop and reset the VREP simulation
    interface.init_position_mode()
    interface.send_target_angles(robot_config.START_ANGLES)
    interface.disconnect()

    print('Simulation terminated...')

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(8,12))
        ax1 = fig.add_subplot(211)
        ax1.set_ylabel('Distance (m)')
        ax1.set_xlabel('Time (ms)')
        ax1.set_title('Distance to target')
        ax1.plot(np.sqrt(np.sum((np.array(target_track) -
                                 np.array(ee_track))**2, axis=1)))

        ax2 = fig.add_subplot(212, projection='3d')
        ax2.set_title('End-Effector Trajectory')
        ax2.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax2.scatter(target_track[0, 0], target_track[0, 1], target_track[0, 2],
                 label='target', c='r')
        ax2.legend()
        ax2.set_xlim(0, 0.5)
        ax2.set_ylim(0, 0.5)
        plt.show()
