"""Uses force control to compensate for gravity.  The arm will
hold its position while maintaining compliance.  """

import numpy as np
import traceback

import abr_jaco2
from abr_control.controllers import Floating

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)
ctrlr = Floating(robot_config)
# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

q_track = []

# connect to the jaco
interface.connect()
interface.init_position_mode()

# Move to home position
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
try:
    interface.init_force_mode()
    while 1:
        feedback = interface.get_feedback()

        u = ctrlr.generate(q=feedback['q'], dq=feedback['dq'])
        interface.send_forces(np.array(u, dtype='float32'))

        # track data
        q_track.append(np.copy(feedback['q']))

except Exception as e:
    print(traceback.format_exc())

finally:
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
