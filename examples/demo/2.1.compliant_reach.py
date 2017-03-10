"""
Demo script, compliant reach to target.
"""
import numpy as np
import redis
import timeit
import traceback

import abr_control
import abr_jaco2
from demo_class import Demo

class Demo21(Demo):
    def __init__(self):

        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config(
            use_cython=True, hand_attached=True)

        super(Demo21, self).__init__()

        # instantiate operation space controller
        self.ctrlr = abr_control.controllers.osc(
            self.robot_config, kp=20, kv=8, vmax=1.0, null_control=False)

        self.robot_config.Tx(
            'camera', x=np.zeros(3), q=np.zeros(6))
        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        zeros = np.zeros(self.robot_config.num_joints)
        self.ctrlr.control(zeros, zeros, np.zeros(3),
            offset=self.robot_config.offset)

        # track data
        self.tracked_data = {'q': [], 'dq': [], 'filtered_target': [],
            'wrist': [], 'offset': [], 'target': []}
        self.redis_server = redis.StrictRedis(host='localhost')
        self.redis_server.set("controller_name", "Compliant")

        self.previous = None

    def start_setup(self):

        # switch to torque control mode
        self.interface.init_force_mode()

        # get position feedback from robot
        self.get_qdq()
        self.filtered_target = self.robot_config.Tx(
            'EE', q=self.q, x=self.robot_config.offset)

    def start_loop(self):
        now = timeit.default_timer()
        if self.previous is not None and self.count % 1000 == 0:
            print("dt:", now - self.previous)
        self.previous = now

        # get position feedback from robot
        self.get_qdq()
        xyz = self.robot_config.Tx('EE', q=self.q, x=self.robot_config.offset)

        # read from vision, update target if new
        # which also does the offset and normalization
        target_xyz = self.get_target_from_camera()
        camera_target = np.copy(target_xyz)
        target_xyz = self.normalize_target(target_xyz)
        # filter the target so that it doesn't jump, but moves smoothly
        # self.filtered_target += .005 * (target_xyz - self.filtered_target)
        self.filtered_target += .01 * (target_xyz - self.filtered_target)

        # generate osc signal
        u = self.ctrlr.control(
            q=self.q, dq=self.dq, target_pos=self.filtered_target,
            offset=self.robot_config.offset)
        u[0] *= 2.0
        # send control signal to Jaco 2
        self.interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if self.count % 100 == 0:
            self.print_error(xyz, target_xyz)
            print('target without normalization: ', camera_target)
            #print('q: ', self.q)

        # track data
        # self.tracked_data['q'].append(np.copy(self.q))
        # self.tracked_data['dq'].append(np.copy(self.dq))
        self.tracked_data['filtered_target'].append(np.copy(self.filtered_target))
        self.tracked_data['wrist'].append(np.copy(
          self.robot_config.Tx('EE', self.q)))
        self.tracked_data['offset'].append(np.copy(xyz))
        self.tracked_data['target'].append(np.copy(self.filtered_target))

try:
    demo = Demo21()
    demo.run()

except:
     print(traceback.format_exc())

finally:
    demo.stop()
    demo.write_data()
