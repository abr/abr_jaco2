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

    def start_loop(self, q, dq):
        self.interface.apply_q(self.demo_pos_q)
        time.sleep(1)

try:
    demo = Demo()
    demo.run()

except Exception as e:
     print(e)

finally:
    demo.stop()
