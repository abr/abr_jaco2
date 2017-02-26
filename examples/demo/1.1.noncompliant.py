"""
Demo script, non-compliant hold position.
"""
import numpy as np
import signal
import sys
import time
import abr_control
import abr_jaco2

kp = 4.0
kv = 2.0
loop_limit = 15000

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    use_cython=True, hand_attached=False)

# NOTE: right now, in the osc when vmax = None, velocity is compensated
# for in joint space, with vmax set it's in task space

# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.osc(
    robot_config, kp=kp, kv=kv, vmax=1.0, null_control=False)
# create signal to compensate for friction
friction = abr_jaco2.signals.friction(robot_config)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(6), np.zeros(6), target_pos=np.zeros(3))
friction.generate(dq=np.zeros(6))

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)
# connect to the jaco
interface.connect()

try:
    # move to the home position
    interface.apply_q(robot_config.demo_pos_q)

    # just hold current position
    while 1:
       time.sleep(1)

except Exception as e:
     print(e)

finally:
    # return back to home position
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position_start)
    # close the connection to the arm
    interface.disconnect()
