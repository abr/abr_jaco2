""" An example script moving the arm to 4 targets in joint space
using position control, _not_ force control. """

import sys
import numpy as np
import time
import traceback

import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

# in radians between 0-2Pi
target_joint_angles = np.array([
    [1.98, 1.86, 2.11, 4.71, 0.0, 3.0],
    [1.57, 2.56, 1.65, 3.42, 0.75, 1.85],
    [0.29, 3.75, 4.78, 4.78, 0.10, 0.0],
    [0.0, 2.42, 2.28, 6.22, 1.3, 1.75]], dtype='float32')

# connect to the jaco and move to it's start rest position
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.START_ANGLES)

q_track = []
target_track = []

try:
    for ii in range(0, len(target_joint_angles)):

        # send target angles to the arm
        interface.send_target_angles(target_joint_angles[ii])
        # the arm will move to the target before returning from the function
        feedback = interface.get_feedback()

        # track data
        q_track.append(np.copy(feedback['q']))
        target_track.append(np.copy(target_joint_angles[ii]))

        # wait for a second before moving to the next target
        time.sleep(1)

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.send_target_angles(robot_config.START_ANGLES)
    interface.disconnect()

    q_track = np.array(q_track)
    target_track = np.array(target_track)
    import matplotlib
    matplotlib.use("TKAgg")
    import matplotlib.pyplot as plt
    plt.figure()
    for ii in range(0, 5):
        plt.subplot(6, 1, ii + 1)
        plt.title('Target vs. Actual Joint Angles')
        plt.xlabel('Target Position')
        plt.ylabel('Joint Position (rad)')
        plt.plot(np.arange(0, len(q_track[:, ii])), q_track[:, ii],
                 label="Actual")
        plt.plot(np.arange(0, len(target_track[:, ii])), target_track[:, ii],
                 '--', label="Target")
        plt.legend()
    plt.show()
    sys.exit()
