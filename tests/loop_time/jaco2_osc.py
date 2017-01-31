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
ctrlr = abr_control.controllers.osc(robot_config, kp=1, vmax=0.5)
# generate the functions we need
ctrlr.control(q=np.zeros(6), dq=np.zeros(6), target_x=np.ones(3))

try:
    interface.connect()
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    # create a target based on initial arm position
    feedback = interface.get_feedback()
    q = (np.array(feedback['q']) % 360) * np.pi / 180.0
    start = robot_config.Tx('EE', q=q)
    target_xyz = start + np.array([-.25, .25, -.1])

    num_timesteps = 1000
    times = np.zeros(num_timesteps)
    interface.init_force_mode()

    for ii in range(num_timesteps):
        start_time = time.time()
        # get arm feedback from VREP
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0
        # use visual feedback to get object endpoint position
        ee_xyz = robot_config.Tx('EE', q=q)

        # generate control signal
        u = ctrlr.control(
            q=q,
            dq=dq,
            target_x=target_xyz,
            target_dx=np.zeros(3))

        # apply the control signal, step the sim forward
        interface.apply_u(np.array(u, dtype="float32"))
        times[ii] = time.time() - start_time

finally:
    # stop and reset the VREP simulation
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    interface.disconnect()

    print('Average loop time: %.5f' % (np.sum(times) / num_timesteps))
