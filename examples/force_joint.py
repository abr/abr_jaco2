"""
A basic script for connecting and moving the arm to 4 targets
in joint space using force control. The joint angles are recorded
and plotted against the target angles once the final target is
reached, and the arm has moved back to its default resting position.

         ----------------- WARNING --------------------

To reach joint angle targets with <2deg of error use gains of kp=20
and kv=6. The arm will remain compliant, but will move fairly quickly

The gains can be dropped (kp=4, kv=2) and the arm will move more slowly
and be more compliant. However, due to the high stiction of the Jaco2
it will not reach with the same accuracy
"""

import sys
import numpy as np
import traceback

from abr_control.controllers import Joint
import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)
ctrlr = Joint(robot_config, kp=25, kv=12)

zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, zeros)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

TARGET_POS = np.array([1.98, 1.86, 2.11, 4.71, 0.0, 3.0], dtype='float32')
TARGET_VEL = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01], dtype='float32')

# connect to the jaco
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

# set up arrays for tracking end-effector and target position
q_track = []

# threshold for a successful reach [ radians ] preset to 10 degrees
thres = 0.17

try:
    interface.init_force_mode()
    ii = 0
    print_counter = 0
    while 1:
        feedback = interface.get_feedback()
        q = feedback['q']
        target_reached = 0
        for jj in range(0, robot_config.N_JOINTS):
            """If current joint angle is < thres from the target it adds to
            the counter, once all joints are within tolerance the arm moves
            to the next target.
            """
            if abs(((TARGET_POS[jj] % 6.28 + 6.28)
                    % 6.28)
                   - q[jj]) < thres:
                target_reached += 1
            elif print_counter % 1000 == 0:
                print('Joint %i not at target angle' % jj)
        print_counter += 1

        if target_reached == robot_config.N_JOINTS - 1:
            break

        u = ctrlr.generate(q=feedback['q'], dq=feedback['dq'],
                          target_pos=TARGET_POS, target_vel=TARGET_VEL)
        interface.send_forces(np.array(u, dtype='float32'))

        q_track.append(np.copy(feedback['q']))

        if print_counter % 1000 == 0:
            print('------------------------')


except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

    q_track = np.array(q_track)
    if q_track.shape[0] > 0:
        import matplotlib
        matplotlib.use("TKAgg")
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot((q_track + np.pi) % (np.pi * 2) - np.pi)
        plt.plot(np.ones(q_track.shape) *
                ((TARGET_POS + np.pi) % (np.pi * 2) - np.pi), '--')
        plt.legend(range(robot_config.N_LINKS))
        plt.tight_layout()
        plt.show()
        print("PLOTTING")
        print('q: ', q_track)
    else:
        print("NO PLOTTING")
