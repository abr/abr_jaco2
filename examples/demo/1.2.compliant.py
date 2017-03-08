"""
Demo script, compliant hold position.
"""
import numpy as np
import redis
import traceback

import abr_control
import abr_jaco2
from demo_class import Demo

class Demo12(Demo):
    def __init__(self):

        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config(
            use_cython=True, hand_attached=True)

        super(Demo12, self).__init__()

        # account for wrist to fingers offset
        self.R_func = self.robot_config._calc_R('EE')
        self.offset = np.array([0.0, 0.0, 0.12])  # 20 cm from wrist

        # instantiate operation space controller
        self.ctrlr = abr_control.controllers.osc(
            self.robot_config, kp=20, kv=4, vmax=1, null_control=False)
        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        zeros = np.zeros(self.robot_config.num_joints)
        self.ctrlr.control(zeros, zeros, np.zeros(3))

        # track data
        self.tracked_data = {'target': [], 'wrist': []}
        self.redis_server = redis.StrictRedis(host='localhost')
        self.redis_server.set("controller_name", "Compliant")

    def start_setup(self):
        self.get_qdq()
        self.filtered_target = self.robot_config.Tx(
            'EE', q=self.q, x=self.offset)
        # switch to torque control mode
        self.interface.init_force_mode()

    def start_loop(self):
        # get position feedback from robot
        self.get_qdq()
        self.filtered_target += .005 * (self.demo_pos_xyz - self.filtered_target)
        xyz = self.robot_config.Tx('EE', q=self.q, x=self.offset)

        # normalized target and incorporate offset
        self.target_subtraction(
            self.demo_pos_xyz, self.offset)

        # generate osc signal
        u = self.ctrlr.control(
            q=self.q, dq=self.dq, target_pos=self.filtered_target)

        # send control signal to Jaco 2
        self.interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if self.count % 100 == 0:
            self.print_error(xyz, self.demo_pos_xyz)
        # track data
        # self.tracked_data['target'].append(self.demo_pos_xyz)
        # self.tracked_data['wrist'].append(self.robot_config.Tx('EE', self.q))

try:
    demo = Demo12()
    demo.run()

except Exception as e:
     print(traceback.format_exc())

finally:
    demo.stop()
    demo.write_data()
