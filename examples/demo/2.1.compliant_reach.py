"""
Demo script, compliant hold position.
"""
import numpy as np
import abr_control
import abr_jaco2
import os

kp = 10.0
kv = 3.0
vmax = 1.0

# targets in camera coordinates
targets = [
    [-0.34, -0.045, 0.69],  # left limit
    [0.24, -0.015, 0.525],  # right limit
    [0.04, 0.295, 0.957],  # middle bottom limit
    [-0.023, -0.121, 0.424],  # middle upper limit
    [0.006, -0.013, 0.682]]  # middle middle
target_xyz = targets[0]

# filename where vision system writes object xyz coordinates
filename = 'data/target_position.txt'

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    use_cython=True, hand_attached=True)

# NOTE: right now, in the osc when vmax = None, velocity is compensated
# for in joint space, with vmax set it's in task space

# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.osc(
    robot_config, kp=kp, kv=kv, vmax=vmax, null_control=False)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(6), np.zeros(6), target_pos=np.zeros(3))

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)
# connect to the jaco
interface.connect()
# move to the home position
interface.apply_q(robot_config.home_position_start)


try:
    # set up key input tracker
    kb = abr_jaco2.KBHit()

    count = 0
    move_home = False
    start_movement = False
    target_xyz = None
    print('Arm Ready')

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
                    # set it so that target is no farther than .8m away
                    norm = np.linalg.norm(target_xyz)
                    if norm > .8:
                        target_xyz = (target_xyz /
                                      np.linalg.norm(target_xyz) * .8)

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
                interface.init_position_mode()
            if ord(c) == 113:  # letter q, quits and goes to finally
               print('Returning to home position')
               break;

except Exception as e:
     print(e)

finally:
    # return back to home position
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position_start)
    # close the connection to the arm
    interface.disconnect()
    kb.set_normal_term()
