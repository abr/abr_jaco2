""" Example script of moving the arm to 3 targets using OSC """

import sys
import numpy as np
import traceback

from abr_control.controllers import OSC
import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)

# instantiate operation space controller
ctrlr = OSC(robot_config, kp=25, kv=5, vmax=1, null_control=False)
# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, np.zeros(3))
# offset to move control point from palm to fingers
robot_config.Tx('EE', q=zeros, x=robot_config.OFFSET)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

target_xyz = np.array([[.57, .03, .87],
                       [.4, -.4, .78],
                       [.467, -.22, .78]], dtype='float32')

# connect to the jaco
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

# set up arrays for tracking end-effector and target position
ee_track = []
target_track = []

try:
    count = 0
    count_at_target = 0 #  must stay at target for 200 loop cycles for success
    target_index = 0

    interface.init_force_mode()
    while target_index < len(target_xyz):
        feedback = interface.get_feedback()
        xyz = robot_config.Tx('EE', q=feedback['q'], x=robot_config.OFFSET)

        # generate the control signal
        u = ctrlr.generate(
            q=feedback['q'], dq=feedback['dq'],
            target_pos=target_xyz[target_index],
            offset=robot_config.OFFSET)

        # additional gain term due to high stiction of jaco base joint
        if u[0] > 0:
            u[0] *= 3.0
        else:
            u[0] *= 2.0

        interface.send_forces(np.array(u, dtype='float32'))
        error = np.sqrt(np.sum((xyz - target_xyz[target_index])**2))

        # print out the error every so often
        if count % 100 == 0:
            print('error: ', error)

        # if within 5cm of target for 200 time steps move to next target
        if error < .05:
            count_at_target += 1
            if count_at_target >= 200:
                count_at_target = 0
                target_index += 1

        # track data
        ee_track.append(np.copy(xyz))
        target_track.append(np.copy(target_xyz[target_index]))

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