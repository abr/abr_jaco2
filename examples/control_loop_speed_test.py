"""
The arm is in operation space control trying to maintain its position. After
10 seconds the arm will return home, after which the average control loop will
be printed, along with a recommendation on whether the computer will be powerful
enough to communicate with the arm at the required speeds.
"""

import numpy as np
import traceback
import timeit

import abr_jaco2
from abr_control.controllers import OSC

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)

ctrlr = OSC(robot_config, kp=20, kv=10)
# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, np.zeros(6))

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

time_track = []

# connect to the jaco
interface.connect()
interface.init_position_mode()

# Move to home position
interface.send_target_angles(robot_config.START_ANGLES)

try:
    print('Running loop speed test for the next 10 seconds...')
    print('During this time the arm will try to maintain its position')
    feedback = interface.get_feedback()
    ee_xyz = np.hstack((robot_config.Tx('EE', q=feedback['q']), np.zeros(3)))
    interface.init_force_mode()
    run_time = 0
    while run_time < 10:
        start = timeit.default_timer()
        feedback = interface.get_feedback()

        u = ctrlr.generate(q=feedback['q'], dq=feedback['dq'], target=ee_xyz)
        interface.send_forces(np.array(u, dtype='float32'))

        # track data
        loop_time = timeit.default_timer() - start
        run_time += loop_time
        time_track.append(np.copy(loop_time))


except Exception as e:
    print(traceback.format_exc())

finally:
    interface.init_position_mode()
    interface.send_target_angles(robot_config.START_ANGLES)
    interface.disconnect()

    time_track = np.array(time_track)
    avg_loop = np.mean(time_track)
    avg_loop_ms = avg_loop * 1000
    if avg_loop > 0.005:
        print('W A R N I N G: You may run into performance issues with your'
              + ' current loop speed of %fms'%avg_loop_ms)
        print('It is not recommended to use force control with a loop speed'
              + ' > 5ms')
    elif avg_loop > 0.0035:
        print('Your average loop speed is %fms' % avg_loop_ms)
        print('For best performance your loop speed should be ~3ms, you may'
              + ' notice minimal loss in performance')
    else:
        print('Your average loop speed is %fms'%avg_loop_ms
              +' and is within the recommended limit')

    # plot joint angles throughout trial
    import matplotlib
    matplotlib.use("TKAgg")
    import matplotlib.pyplot as plt
    plt.figure()
    plt.title('Loop Speed')
    plt.ylabel('time [sec]')
    plt.plot(time_track, label='Average: %fms' %avg_loop_ms)
    plt.legend()
    plt.show()
