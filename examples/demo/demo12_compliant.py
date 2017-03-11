"""
Demo script, compliant hold position.
"""
import numpy as np
import traceback

import abr_control
import abr_jaco2
from demo_class import Demo


class Demo12(Demo):
    def __init__(self, track_data=False):

        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config(
            use_cython=True, hand_attached=True)

        super(Demo12, self).__init__(track_data)

        # account for wrist to fingers offset
        self.R_func = self.robot_config._calc_R('EE')

        # instantiate operation space controller
        self.ctrlr = abr_control.controllers.osc(
            self.robot_config, kp=20, kv=6, vmax=1, null_control=False)
        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        zeros = np.zeros(self.robot_config.num_joints)
        self.ctrlr.control(zeros, zeros, np.zeros(3))
        self.robot_config.Tx('EE', q=zeros, x=self.robot_config.offset)

        # track data
        if self.track_data is True:
            self.tracked_data = {'target': [], 'EE': []}

        # set target for vrep display
        self.redis_server.set(
            'norm_target_xyz_robot_coords', '%.3f %.3f %.3f' %
            tuple(self.demo_pos_xyz))

    def start_setup(self):
        self.get_qdq()
        self.filtered_target = self.robot_config.Tx(
            'EE', q=self.q, x=self.robot_config.offset)
        # switch to torque control mode
        self.interface.init_force_mode()

    def start_loop(self):
        # get position feedback from robot
        self.get_qdq()
        self.filtered_target += .01 * (
            self.demo_pos_xyz - self.filtered_target)
        xyz = self.robot_config.Tx('EE', q=self.q, x=self.robot_config.offset)

        # generate osc signal
        u = self.ctrlr.control(
            q=self.q, dq=self.dq, target_pos=self.filtered_target)
        u[0] *= 2.0
        # send control signal to Jaco 2
        self.interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if self.count % 100 == 0:
            self.print_error(xyz, self.demo_pos_xyz)

        if self.track_data is True:
            self.tracked_data['target'].append(self.filtered_target)
            self.tracked_data['EE'].append(xyz)

try:
    demo = Demo12()
    demo.run()

except Exception as e:
    print(traceback.format_exc())

finally:
    demo.stop()