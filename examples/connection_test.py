""" Tests the basic connection and disconnect from the Jaco2 """

import numpy as np
import time

import abr_control
import abr_jaco2

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config()
# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)

try:
    interface.connect()

except Exception as e:
    print(e)

finally:
    interface.disconnect()
