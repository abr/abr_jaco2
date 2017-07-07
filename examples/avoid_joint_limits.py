""" Floating controller with obstacle avoidance for joint limits

The arm will remain compliant and float in its current position,
until it approaches the specified joint limits
"""

import numpy as np
import traceback

import abr_jaco2
from abr_control.controllers import Floating, signals

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)

interface = abr_jaco2.Interface(robot_config)
# connect to the jaco and initialize position mode
interface.connect()
interface.init_position_mode()

# create our environment
ctrlr = Floating(robot_config)
ctrlr.generate(np.zeros(6), np.zeros(6))

avoid = signals.AvoidJointLimits(
    robot_config,
    min_joint_angles=[None, 1.57, 1.57, None, None, None],
    max_joint_angles=[None, 4.71, 4.71, None, None, None],
    max_torque=[5]*robot_config.N_JOINTS)

# send to start position so we can switch to torque mode
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
interface.init_force_mode()
try:
    while 1:
        feedback = interface.get_feedback()

        u = ctrlr.generate(q=feedback['q'], dq=feedback['dq'])
        # add in joint limit avoidance
        u += avoid.generate(q)
        # apply the control signal
        interface.send_forces(u)

        # track data
        q_track.append(np.copy(feedback['q']))

except:
    print(traceback.format_exc())

finally:
    # switch to position mode, go to rest position and disconnect
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

    # plot joint angles throughout trial
    q_track = np.array(q_track)
    import matplotlib
    matplotlib.use("TKAgg")
    import matplotlib.pyplot as plt
    plt.figure()
    plt.title('Joint Angles')
    plt.plot(q_track)
    plt.legend(range(robot_config.N_JOINTS))
    plt.show()
