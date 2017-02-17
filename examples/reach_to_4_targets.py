"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np
import signal
import sys

import abr_control
import abr_jaco2

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    regenerate_functions=False, use_cython=True,
    hand_attached=False)

# NOTE: right now, in the osc when vmax = None, velocity is compensated
# for in joint space, with vmax set it's in task space

# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.osc(
    robot_config, kp=4.0, kv=2.0, vmax=None, null_control=False)
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
# move to the home position
interface.apply_q(robot_config.home_position_start)

# set up arrays for tracking end-effector and target position
ee_track = []
u_track = []
# q_track = []
# dq_track = []
targets_track = []

count = 0
target_index = 0
at_target_count = 0

# list of targets to move to
targets = [[-.4, .2, .70],
           [-.467, -.22, .78],
           [.467, -.22, .78],
           [.467, .22, .78],
           [-.467, .22, .78]]
target_xyz = targets[0]
print('Moving to first target: ', target_xyz)

import time
print('THREE')
time.sleep(1)
print('TWO')
time.sleep(1)
print('ONE')
time.sleep(1)

# switch to torque control mode
interface.init_force_mode()


def on_exit(signal, frame):
    """ A function for plotting the end-effector trajectory and error """
    global ee_track, target_track, u_track
    ee_track = np.array(ee_track)
    target_track = np.array(target_track)
    u_track = np.array(u_track)

    import matplotlib.pyplot as plt

    # plot targets and trajectory of end-effectory in 3D
    abr_control.utils.plotting.plot_trajectory(ee_track, targets_track)

    plt.figure()
    plt.plot(np.array(u_track))
    # plt.subplot(2, 1, 1)
    # plt.plot(np.array(q_track))
    # plt.subplot(2, 1, 2)
    # plt.plot(np.array(dq_track), '--')
    plt.show()
    sys.exit()

# call on_exit when ctrl-c is pressed
signal.signal(signal.SIGINT, on_exit)

# try:
while 1:
    feedback = interface.get_feedback()
    q = np.array(feedback['q'])
    dq = np.array(feedback['dq'])

    u = ctrlr.control(q=q, dq=dq, target_pos=target_xyz)
    friction_generated = friction.generate(dq=dq)
    u += friction_generated
    interface.send_forces(np.array(u, dtype='float32'))

    hand_xyz = robot_config.Tx('EE', q=q)
    error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
    if error < .01:
        # if we're at the target, start count
        # down to moving to the next target
        at_target_count += 1
        if at_target_count >= 200:
            target_index += 1
            if target_index > len(targets):
                break
            else:
                target_xyz = targets[target_index]
                print('Moving to next target: ', target_xyz)
            at_target_count = 0

    ee_track.append(hand_xyz)
    u_track.append(np.copy(u))
    # q_track.append(np.copy(q))
    # dq_track.append(np.copy(dq))
    targets_track.append(target_xyz)
    count += 1
    if count % 100 == 0:
        print('error: ', error)

# except Exception as e:
#     print(e)
#
# finally:
# return back to home position
interface.init_position_mode()
interface.apply_q(robot_config.home_position_end)
# close the connection to the arm
interface.disconnect()
