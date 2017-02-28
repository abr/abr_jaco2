"""
Demo script, non-compliant hold position.
"""
import time

import abr_jaco2
from demo import Demo

class Demo11(Demo):
    def __init__(self):
        super(Demo11, self).__init__()

        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config(
            use_cython=True, hand_attached=True)
        self.target_xyz = robot_config.demo_pos_xyz

        # instantiate the REACH controller for the jaco2 robot
        self.ctrlr = abr_control.controllers.osc(
            robot_config, kp=10, kv=3, vmax=1, null_control=False)
        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        self.ctrlr.control(np.zeros(6), np.zeros(6), target_pos=np.zeros(3))

    def generate_u(self, q, dq):
        u = ctrlr.control(q=q, dq=dq, target_pos=self.target_xyz)
        # send control signal to Jaco 2
        interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if self.count % 100 == 0:
            hand_xyz = self.robot_config.Tx('EE', q=q)
            error = np.sqrt(np.sum((hand_xyz - self.target_xyz)**2))
            print('error: ', error)

try:
    demo = Demo()
    demo.run()

except Exception as e:
     print(e)

finally:
    demo.stop()
