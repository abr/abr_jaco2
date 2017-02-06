"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np

import abr_control
import abr_jaco2

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    regenerate_functions=True, use_cython=True,
    use_simplify=False, hand_attached=False)
# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.osc(
    robot_config, kp=10.0, kv=3.0, vmax=1)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(6), np.zeros(6), target_x=np.zeros(3))

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)
# connect to the jaco
interface.connect()
# move to the home position
interface.apply_q(robot_config.home_position)
# switch to torque control mode
interface.init_force_mode()

# set up arrays for tracking end-effector and target position
ee_track = []
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

try:
    ctr = 0
    while 1:
        ctr += 1
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0

        u = ctrlr.control(q=q, dq=dq, target_x=target_xyz)
        interface.apply_u(np.array(u, dtype='float32'))

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
        targets_track.append(target_xyz)
        count += 1
        if count %100 == 0:
            print('error: ', error)

except Exception as e:
    print(e)

finally:
    # return back to home position
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    # close the connection to the arm
    interface.disconnect()

    if count > 0:  # i.e. if it successfully ran
        import matplotlib.pyplot as plt
        import seaborn

        ee_track = np.array(ee_track)
        targets_track = np.array(targets_track)
        # plot targets and trajectory of end-effectory in 3D
        abr_control.utils.plotting.plot_trajectory(ee_track, targets_track)

        plt.tight_layout()
        plt.show()
