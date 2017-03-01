"""
Demo script, non-compliant hold position.
"""
import time

import abr_jaco2
from demo_class import Demo

class Demo11(Demo):
    def __init__(self):
        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config(
            use_cython=True, hand_attached=True)

        super(Demo11, self).__init__()

        self.target_xyz = self.robot_config.demo_pos_xyz
        self.moved = False

    def start_setup(self):
        # switch to position control mode
        self.interface.init_position_mode()

    def start_loop(self):
        # TODO: possibly update this so it can move to target
        # position more than once per run
        if self.moved is False:
            self.interface.apply_q(self.robot_config.demo_pos_q)
            self.moved = True
        time.sleep(1)

try:
    demo = Demo11()
    demo.run()

except Exception as e:
     print(e)

finally:
    demo.stop()
