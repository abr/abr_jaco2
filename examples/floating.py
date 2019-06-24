"""Uses force control to compensate for gravity.  The arm will
hold its position while maintaining compliance.  """

import numpy as np
import traceback
import time

import abr_jaco2
from abr_control.controllers import Floating

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)
ctrlr = Floating(robot_config, dynamic=True, task_space=True)
# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

q_track = []
u_track = []
q_T_track = []

# connect to the jaco
interface.connect()
interface.init_position_mode()

# Move to home position
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
try:
    interface.init_force_mode()
    cnt = 0
    while cnt < 3000:
        now = time.time()
        feedback = interface.get_feedback()
        q_T = interface.get_torque_load()

        u = ctrlr.generate(q=feedback['q'], dq=feedback['dq'])
        interface.send_forces(np.array(u, dtype='float32'))

        # track data
        q_track.append(np.copy(feedback['q']))
        u_track.append(np.copy(u))
        q_T_track.append(np.copy(q_T))
        cnt += 1

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
    col = ['r', 'b', 'g', 'y', 'k', 'm']
    col2 = ['r--', 'b--', 'g--', 'y--', 'k--', 'm--']
    fig = plt.figure()
    a1 = fig.add_subplot(211)
    a1.set_title('Joint Status')
    a1.set_ylabel('Torque [Nm]')
    a2 = fig.add_subplot(212)
    a2.set_ylabel('Position [rad]')
    u_track = np.array(u_track).T
    q_T_track = np.array(q_T_track).T
    for ii in range(0,6):
        a1.plot(u_track[ii], col2[ii], label='u%i'%ii)
        a2.plot(q_T_track[ii], col[ii], label='q%i'%ii)
    a1.legend()
    a2.legend()
    plt.show()
