"""
Demo script, compliant hold position.
"""
import numpy as np
import signal
import time
import abr_control
import abr_jaco2
import os

kp = 10.0
kv = 3.0
vmax = 1.0
targets = [[.4, .2, .70],
           [.3, -.22, .60],
           [.45, 0.0, .65]]
target_xyz = targets[0]
q_track =[]
dq_track =[]
filename = 'data/target_position.txt'
# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    use_cython=True, hand_attached=True)
robot_config.generate_control_functions()

# NOTE: right now, in the osc when vmax = None, velocity is compensated
# for in joint space, with vmax set it's in task space

# instantiate the REACH controller for the jaco2 robot
floating_ctrlr = abr_control.controllers.floating(
    robot_config)
ctrlr = abr_control.controllers.osc(
    robot_config, kp=kp, kv=kv, vmax=vmax, null_control=False)

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)
# connect to the jaco
interface.connect()
# move to the home position
interface.apply_q(robot_config.home_position_start)
move_home = False
target_xyz = None


try:

    kb = abr_jaco2.KBHit()
    count = 0
    start_movement = False
    while 1:
        if start_movement is True:

            # get feedback
            feedback = interface.get_feedback()
            q = np.array(feedback['q'])
            dq = np.array(feedback['dq'])

            # get the target location from camera
            if count % 125 == 0:
                # check that file isn't empty
                if os.path.getsize(filename) > 0:
                    # read the target position
                    with open(filename, 'r') as f:
                        camera_xyz = (f.readline()).split(',')
                    # cast as floats
                    camera_xyz = np.array(
                        [float(i) for i in camera_xyz],
                        dtype='float32')
                    # transform from camera to robot reference frame
                    target_xyz = robot_config.Tx(
                        'camera', x=camera_xyz, q=np.zeros(6))
                    print('target position: ', target_xyz)

            if target_xyz is None:
                # until there's a target float
                u = floating_ctrlr.control(q=q, dq=dq)
            else:
                # once a target is read drive arm to target
                u = ctrlr.control(q=q, dq=dq, target_pos=target_xyz)

            # send control signal to Jaco 2
            interface.send_forces(np.array(u, dtype='float32'))

            # print out the error every so often
            if count % 100 == 0:
                hand_xyz = robot_config.Tx('EE', q=q)
                error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
                print('error: ', error)
            count += 1

        if move_home is True:
            interface.apply_q(robot_config.home_position_start)
            move_home = False

        if kb.kbhit():
            c = kb.getch()
            if ord(c) == 112:  # letter p, closes hand
                interface.open_hand(False)
            if ord(c) == 111:  # letter o, opens hand
                interface.open_hand(True)
            if ord(c) == 115:  # letter s, starts movement
                start_movement = True
                # switch to torque control mode
                interface.init_force_mode()
            if ord(c) == 104:  # letter h, move to home
                start_movement = False
                move_home = True
                # switch to position control mode
                interface.init_posiition_mode()

except Exception as e:
     print(e)

finally:
    # return back to home position
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position_start)
    # close the connection to the arm
    interface.disconnect()
    kb.set_normal_term()

    #np.savez_compressed('q', q=q_track)
    #np.savez_compressed('dq', dq=dq_track)
