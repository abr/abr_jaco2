"""
Demo script, non-compliant hold position.
"""
import time

import abr_jaco2
import demo_class

class Demo11(demo_class.Demo):
    def __init__(self):
        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config(
            use_cython=True, hand_attached=True)

        super(Demo11, self).__init__()

        self.target_xyz = self.robot_config.demo_pos_xyz

    def start_loop(self):
        self.interface.apply_q(self.demo_pos_q)
        time.sleep(1)

try:
    demo = Demo11()
    demo.run()

except Exception as e:
     print(e)

finally:
    demo.stop()
