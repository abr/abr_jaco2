"""
Demo script, compliant hold position.
"""
import numpy as np
import signal
import sys
import time
import abr_control
import abr_jaco2

kp = 10.0
kv = 3.0
q_track =[]
dq_track =[]

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    use_cython=True, hand_attached=True)

# NOTE: right now, in the osc when vmax = None, velocity is compensated
# for in joint space, with vmax set it's in task space

# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.osc(
    robot_config, kp=kp, kv=kv, vmax=1.0, null_control=False)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(6), np.zeros(6), target_pos=np.zeros(3))

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)
# connect to the jaco
interface.connect()
# move to the home position
interface.apply_q(robot_config.home_position_start)
# switch to torque control mode
interface.init_force_mode()

try:
  while 1:
        feedback = interface.get_feedback()
        q = np.array(feedback['q'])
        dq = np.array(feedback['dq'])
        u = ctrlr.control(q=q, dq=dq, target_pos=robot_config.demo_pos_xyz)
        interface.send_forces(np.array(u, dtype='float32'))
        #q_track.append(q)
        #dq_track.append(dq)

except Exception as e:
     print(e)
finally:
    # return back to home position
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position_end)
    # close the connection to the arm
    interface.disconnect()

    #np.savez_compressed('q', q=q_track)
    #np.savez_compressed('dq', dq=dq_track)