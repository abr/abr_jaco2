""" Tests the speed of the osc controller running on the Jaco2. """

import numpy as np
import time

import abr_control
import abr_jaco2

# initialize our robot config for the ur5
robot_config = abr_jaco2.robot_config(
    regenerate_functions=True, use_cython=True,
    use_simplify=False, hand_attached=False)

# create our VREP interface
interface = abr_jaco2.interface(robot_config)

# instantiate the REACH controller with obstacle avoidance
ctrlr = abr_control.controllers.floating(robot_config)
# generate the functions we need
ctrlr.control(np.zeros(6), np.zeros(6))

try:
    interface.connect()
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    # create a target based on initial arm position

    num_timesteps = 1000
    times = np.zeros(num_timesteps)
    interface.init_force_mode()

    for ii in range(num_timesteps):
        start_time = time.time()
        # get arm feedback from VREP
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0

        # generate control signal
        u = ctrlr.control(q=q, dq=dq)

        # apply the control signal, step the sim forward
        interface.apply_u(np.array(u, dtype="float32"))
        times[ii] = time.time() - start_time
finally:
    # stop and reset the VREP simulation
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    interface.disconnect()

    print('Average loop time: %.5f' % (np.sum(times) / num_timesteps))
