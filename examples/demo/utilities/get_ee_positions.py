"""
Puts arm into floating controller, press w to record a position,
press q to quit and print out all selected positions
"""
import numpy as np
import traceback

import abr_jaco2
import abr_control

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    use_cython=True, hand_attached=True)

# instantiate operation space controller
ctrlr = abr_control.controllers.floating(robot_config)

# make sure all the functions we need are generated
zeros = np.zeros(robot_config.num_joints)
ctrlr.control(zeros, zeros)
robot_config.Tx('EE', q=zeros, x=robot_config.offset)

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)
# connect to the jaco
interface.connect()
interface.init_position_mode()
# move to the home position
interface.apply_q(robot_config.init_torque_position)

print('Arm Ready')

# set up key input tracker
kb = abr_jaco2.KBHit()

mode = ''
count = 0
ee_xyz = []
camera_xyz = []
try:
    interface.init_force_mode()
    print('Press \'w\' to record current ee position')
    print('Press \'q\' to quit and print recorded targets')
    while 1:
        if kb.kbhit():
            c = kb.getch()
            if ord(c) == 119:  # letter w, records ee position
                print('Recording EE position')
                mode = 'get_ee_xyz'
            if ord(c) == 113:  # letter q, quits and goes to finally
                print('Returning to home position')
                mode = 'quit'

        feedback = interface.get_feedback()
        q = np.array(feedback['q'])
        dq = np.array(feedback['dq'])
        u = ctrlr.control(q=q, dq=dq)
        interface.send_forces(np.array(u, dtype='float32'))

        if mode == 'get_ee_xyz':
            # get current position of end-effector
            xyz = robot_config.Tx('EE', q=q, x=robot_config.offset)
            ee_xyz.append(np.copy(xyz))
            # convert to camera coordinates
            T_inv = robot_config.T_inv('camera', q=np.zeros)
            camera_xyz.append(np.dot(T_inv, np.hstack([xyz, 1]))[:-1])

            mode = ''

        elif mode == 'quit':
            break

except:
    print(traceback.format_exc())

finally:
    interface.init_position_mode()
    interface.apply_q(robot_config.init_torque_position)
    interface.disconnect()
    print('T A R G E T  P O S I T I O N S')
    print(ee_xyz)
    print('T A R G E T  P O S I T I O N S  C A M E R A')
    print(camera_xyz)
    print('Disconnected')
