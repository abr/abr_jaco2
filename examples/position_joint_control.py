"""
A basic script for connecting and moving the arm to 4 targets
in joint space. The joint angles are recorded and plotted against
the target angles once the final target is reached, and the arm
has moved back to its default resting position.
"""
import sys
import numpy as np
import time
import traceback

import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

# in radians between 0-2Pi
TARGET_POS = np.array([[1.98, 1.86, 2.11, 4.71, 0.0, 3.0],
                       [1.57, 2.56, 1.65, 3.42, 0.75, 1.85],
                       [0.29, 3.75, 4.78, 4.78, 0.10, 0.0],
                       [0.0, 2.42, 2.28, 6.22, 1.3, 1.75]], dtype='float32')

# connect to the jaco and move to it's start  rest position
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

q_track = []

try:
    for ii in range(0,len(TARGET_POS)):
        interface.send_target_angles(TARGET_POS[ii])
        feedback = interface.get_feedback()
        q_track.append(np.copy(feedback['q']))
        time.sleep(1)

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

    q_track = np.array(q_track)
    #TODO: fix plotting, y axis missing
    import matplotlib.pyplot as plt
    plt.figure()
    for ii in range (0, 5):
        plt.subplot(6,1,ii+1)
        plt.title('Target vs. Actual Joint Angles')
        plt.xlabel('Target Position')
        plt.ylabel('Joint Position (rad)')
        plt.plot(np.arange(0,len(TARGET_POS)), q_track[:, ii])
        #plt.gca().set_color_cycle(None)
        plt.plot(np.arange(0,len(TARGET_POS)), TARGET_POS[:, ii], '--')
    plt.tight_layout()
    plt.show()
    sys.exit()
