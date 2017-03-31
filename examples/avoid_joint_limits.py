"""
floating controller with obstacle avoidance for joint limits
"""
import numpy as np
import traceback

import abr_jaco2
import abr_control

# initialize our robot config for the ur5
robot_config = abr_jaco2.config(
    use_cython=True, hand_attached=True)

interface = abr_jaco2.interface(robot_config)
interface.connect()
interface.init_position_mode()

init_torque_position = np.array(
    [0.0, 2.79, 2.72, 4.71, 0.0, 3.04], dtype="float32")
# create our environment
ctrlr = abr_control.controllers.floating(robot_config)
ctrlr.control(np.zeros(6), np.zeros(6))

avoid = abr_control.controllers.signals.avoid_joint_limits(
    robot_config,
    min_joint_angles=[None, 0.87, 0.33, None, None, None],
    max_joint_angles=[None, 5.41, 5.95, None, None, None],
    max_torque=5)

interface.send_target_angles(init_torque_position)
interface.init_force_mode()
try:
    while 1:
        feedback = interface.get_feedback()
        q = np.array(feedback['q'])
        dq = np.array(feedback['dq'])

        u = ctrlr.control(q=q, dq=dq)

        # add in joint limit avoidance
        #print(avoid.generate(feedback['q']))
        u += avoid.generate(q)

        # apply the control signal, step the sim forward
        interface.send_forces(u)

finally:
    # stop and reset the simulation
    interface.init_position_mode()
    interface.send_target_angles(init_torque_position)
    interface.disconnect()
