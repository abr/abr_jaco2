"""
send joint angles and target information to Vrep to replay the run
with data saved in 'file_location'
"""
import numpy as np
import time

import abr_control

file_location = '../data/parameter_testing/compliant/baseline/trial0'
loop_delay = 0.0015

q = np.load('%s/q.npz' % file_location)['arr_0']
filtered_target = np.load('%s/filtered_target.npz' % file_location)['arr_0']

# create our config, interface, and connect to VREP
rc = abr_control.arms.jaco2.config(
    use_cython=True, hand_attached=True)
interface = abr_control.interfaces.vrep(rc)

try:
    interface.connect()

    for ii in range(0, len(q)):
        interface.set_position(q[ii])
        interface.set_xyz('target', filtered_target[ii])

        time.sleep(loop_delay)

finally:
    interface.disconnect()
