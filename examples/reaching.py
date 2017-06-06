"""
A basic script for connecting and moving the arm to 4 targets
in operational space using force control.
"""

import sys
import numpy as np
import traceback

from abr_control.controllers import OSC
import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)

# account for wrist to fingers offset
R_func = robot_config._calc_R('EE')

# instantiate operation space controller
ctrlr = OSC(robot_config, kp=25, kv=5, vmax=1, null_control=False)
# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, np.zeros(3))

robot_config.Tx('EE', q=zeros, x=robot_config.OFFSET)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

TARGET_XYZ = np.array([[.57, .03, .87],
                       [.4, -.4, .78],
                       [.467, -.22, .78]], dtype='float32')
TARGET_VEL = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01], dtype='float32')

# connect to the jaco
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

# set up arrays for tracking end-effector and target position
ee_track = []
target_track = []

try:
    interface.init_force_mode()
    ii = 0
    count = 0
    at_target_count = 0 #  must stay at target for 200 loop cycles for success

    while ii < len(TARGET_XYZ):
        feedback = interface.get_feedback()
        q = feedback['q']
        dq = feedback['dq']

        xyz = robot_config.Tx('EE', q=q, x=robot_config.OFFSET)

        # generate the control signal
        u = ctrlr.generate(q=q, dq=dq, target_pos=TARGET_XYZ[ii],
                                offset=robot_config.OFFSET)

        # additional gain term due to high stiction of jaco base joint
        if u[0] > 0:
            u[0] *= 3.0
        else:
            u[0] *= 2.0

        interface.send_forces(np.array(u, dtype='float32'))
        error = np.sqrt(np.sum((xyz - TARGET_XYZ[ii])**2))

        # print out the error every so often
        if count % 100 == 0:
            print('error: ', error)

        # track data
        ee_track.append(np.copy(xyz))
        target_track.append(np.copy(TARGET_XYZ[ii]))

        if error < .05:
            at_target_count += 1
            if at_target_count >= 200:
                at_target_count = 0
                ii += 1
        count+=1

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib
        matplotlib.use("TKAgg")
        import matplotlib.pyplot as plt
        from abr_control.utils.plotting import plot_3D

        plt.figure()
        plt.plot(np.sqrt(np.sum((np.array(target_track)
                                 - np.array(ee_track))**2, axis=1)))
        plt.ylabel('Distance (m)')
        plt.xlabel('Time (ms)')
        plt.title('Distance to target')

        plot_3D(ee_track, target_track)
        plt.show()
