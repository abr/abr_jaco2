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

import abr_control.controllers.joint as Joint
import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)
ctrlr = Joint(robot_config, kp=20, kv=6)

zeros = np.zeros(robot_config.NUM_JOINTS)
ctrlr.control(zeros, zeros, zeros)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

TARGET_POS = np.array([[1.98, 1.86, 2.11, 4.71, 0.0, 3.0],
                       [1.57, 2.56, 1.65, 3.42, 0.75, 1.85],
                       [0.29, 3.75, 4.78, 4.78, 0.10, 0.0],
                       [0.0, 2.42, 2.28, 6.22, 1.3, 1.75]], dtype='float32')
TARGET_VEL = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01], dtype='float32')

# connect to the jaco
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

# set up arrays for tracking end-effector and target position
q_track = []

try:
    interface.init_force_mode()
    ii = 0
    print_counter = 0
    while ii < len(TARGET_POS):
        feedback = interface.get_feedback()
        q = feedback['q']
        target_reached = 0
        for jj in range(0, robot_config.NUM_JOINTS):
            """If current joint angle is < 2degrees from the target it adds to
            the counter, once all joints are within tolerance the arm moves
            to the next target.
            """
            if abs(((TARGET_POS[ii,jj] % 6.28 + 6.28) % 6.28) - q[jj]) < 0.035:
                target_reached += 1
            elif print_counter%1000 ==0:
                print('Joint %i not at target angle' % jj)
        print_counter += 1

        if target_reached == robot_config.NUM_JOINTS - 1:
            ii += 1

        u = ctrlr.control(q=feedback['q'], dq=feedback['dq'],
                target_pos=TARGET_POS[ii], target_vel=TARGET_VEL)
        interface.send_forces(np.array(u, dtype='float32'))

        q_track.append(np.copy(feedback['q']))

        if print_counter%1000 == 0:
            print('------------------------')


except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

    q_track = np.array(q_track)

    import matplotlib.pyplot as plt

    plt.plot(q_track)
    plt.gca().set_color_cycle(None)
    plt.plot(np.ones(q_track.shape) *
             ((target_pos + np.pi) % (np.pi * 2) - np.pi), '--')
    plt.legend(range(6))
    plt.tight_layout()
    plt.show()
    sys.exit()
