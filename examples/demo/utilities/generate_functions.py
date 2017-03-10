import numpy as np

import abr_control
import abr_jaco2

rc = abr_jaco2.robot_config_neural_1_3(
    use_cython=True, hand_attached=True)

ctrlr = abr_control.controllers.osc(rc)
zeros = np.zeros(rc.num_joints)
ctrlr.control(zeros, zeros, np.zeros(3))
