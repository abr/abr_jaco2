"""
Demo script, compliant reach to target.
"""
import numpy as np

import abr_control
import abr_jaco2
from demo_class import Demo

class Demo21(Demo):
    def __init__(self):

        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config(
            use_cython=True, hand_attached=True)

        super(Demo21, self).__init__()

        # account for wrist to fingers offset
        self.R_func = self.robot_config._calc_R('EE')
        self.fingers_offset = np.array([0.0, 0.0, 0.0])  # 20 cm from wrist

        # instantiate operation space controller
        self.ctrlr = abr_control.controllers.osc(
            self.robot_config, kp=20, kv=4, vmax=1, null_control=False)
        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        zeros = np.zeros(self.robot_config.num_joints)
        self.ctrlr.control(zeros, zeros, np.zeros(3))

        # track data
        self.tracked_data = {'q': [], 'dq': []}
        self.tracked_data['target'] = []
        self.tracked_data['wrist'] = []

    def start_setup(self):
        # switch to torque control mode
        self.interface.init_force_mode()

    def start_loop(self):
        # get position feedback from robot
        self.get_qdq()

        # read from vision, update target if new
        # which also does the offset and normalization
        self.get_target_from_camera()

        # generate osc signal
        u = self.ctrlr.control(
            q=self.q, dq=self.dq, target_pos=self.target_xyz)
        # send control signal to Jaco 2
        self.interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if self.count % 100 == 0:
            self.print_error()

        # track data
        self.tracked_data['q'].append(np.copy(self.q))
        self.tracked_data['dq'].append(np.copy(self.dq))
        self.tracked_data['target'].append(self.target_xyz)
        self.tracked_data['wrist'].append(self.robot_config.Tx('EE', self.q))
try:
    demo = Demo21()
    demo.run()

except Exception as e:
     print(e)

finally:
    demo.stop()
    demo.write_data()
