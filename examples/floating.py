"""Uses force control to compensate for gravity

arm will hold its position while maintaining compliance"""

import numpy as np
import traceback

import abr_jaco2
from abr_control.controllers import Floating

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)
ctrlr = Floating(robot_config)
ctrlr.generate(np.zeros(6), np.zeros(6))

interface = abr_jaco2.Interface(robot_config, use_redis=False)

interface.connect()
interface.init_position_mode()

# Move to home position
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
try:
    interface.init_force_mode()

    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        q = np.array(feedback['q'])
        dq = np.array(feedback['dq'])

        u = ctrlr.generate(q=q, dq=dq)

        interface.send_forces(np.array(u, dtype='float32'))

except Exception as e:
    print(traceback.format_exc())

finally:
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()
