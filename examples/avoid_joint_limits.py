"""
floating controller with obstacle avoidance for joint limits

The arm will remain compliant and float in its current position,
unless it approaches the set joint limits which it will avoid
"""

import numpy as np
import traceback

import abr_jaco2
from abr_control.controllers import Floating, signals

# initialize our robot config for the ur5
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)

interface = abr_jaco2.Interface(robot_config)
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

interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
interface.init_force_mode()
try:
    while 1:
        feedback = interface.get_feedback()
        q = np.array(feedback['q'])
        dq = np.array(feedback['dq'])

        u = ctrlr.generate(q=q, dq=dq)

        # add in joint limit avoidance
        u += avoid.generate(q)

        # apply the control signal, step the sim forward
        interface.send_forces(u)

except:
    print(traceback.format_exc())

finally:
    # stop and reset the simulation
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()
