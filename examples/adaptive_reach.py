"""
Move the jaco2 to a target position with an adaptive controller
that will account for a 2lb weight in its hand

For this script a 2lb dumbbell was placed in the jaco2's hand. You can control
the jaco's hand with the 'hand_control.py' script to place the mass in the
gripper.

The adaptive controller will be able to account for other forces, however the
combined mass of the hand and the 2lb dumbbell are close to the limit of the
jaco2 and it is not recommended to use larger masses.

The adaptive controller takes in the joint positions and velocities for the
base three joints (base, shoulder, elbow) and outputs a control signal [Nm] for
the respective three joints.
"""

import numpy as np
import os
import timeit
import traceback

from abr_control.controllers import OSC, signals, path_planners
import abr_jaco2

plot_error = True

# initialize our robot config
robot_config = abr_jaco2.Config(use_cython=True, hand_attached=True)

# instantiate controller
ctrlr = OSC(robot_config, kp=20, kv=6, vmax=1, null_control=True)

# instantiate path planner and set parameters
n_timesteps = 4000
w = 1e4
zeta = 2
dt = 0.003
path = path_planners.SecondOrder(robot_config, w=w, zeta=zeta, dt=dt,
                                 n_timesteps=n_timesteps)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, np.zeros(3))

robot_config.Tx('EE', q=zeros, x=robot_config.OFFSET)

interface = abr_jaco2.Interface(robot_config)

target_xyz = np.array([.03, -.57, .87])

time_limit = 30 # in seconds

# create our adaptive controller
adapt = signals.DynamicsAdaptation(
    n_input=6,
    n_output=3,
    n_neurons=1000,
    pes_learning_rate=5e-6,
    intercepts=(-0.1, 1.0),
    weights_file=None,
    backend='nengo')

# connect to and initialize the arm
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

if plot_error:
    error_track = []

try:
    interface.init_force_mode()

    # get the end-effector's initial position
    feedback = interface.get_feedback()
    count = 0
    loop_time = 0

    # get joint angle and velocity feedback
    feedback = interface.get_feedback()
    # calculate end-effector position
    ee_xyz = robot_config.Tx('EE', q=feedback['q'], x=robot_config.OFFSET)

    target = np.hstack((ee_xyz, np.zeros(3)))

    while loop_time < time_limit:
        start = timeit.default_timer()

        # get next step along trajectory
        target = path.step(state=target, target=target_xyz)

        feedback = interface.get_feedback()
        q = feedback['q']
        dq = feedback['dq']

        ee_xyz = robot_config.Tx('EE', q=q, x=robot_config.OFFSET)

        # calculate the control signal
        u_base = ctrlr.generate(
            q=q,
            dq=dq ,
            target_pos=target[:3],
            target_vel=target[3:],
            offset = robot_config.OFFSET)

        # adjust for some stiction in the base
        if u_base[0] > 0:
            u_base[0] *= 3.0
        else:
            u_base[0] *= 2.0

        training_signal = ctrlr.training_signal[:3]

        # calculate teh adaptive control signal
        u_adapt = adapt.generate(
                    input_signal=np.concatenate(
                        (robot_config.scaledown('q',q)[:3],
                         robot_config.scaledown('dq',dq)[:3]), axis=0),
                    training_signal=training_signal[:3])

        u = u_base + np.concatenate((u_adapt, np.array([0,0,0])), axis=0)

        interface.send_forces(np.array(u, dtype='float32'))
        error = np.sqrt(np.sum((ee_xyz - target_xyz)**2))

        end = timeit.default_timer() - start
        loop_time += end

        if plot_error:
            error_track.append(np.copy(error))

        if count % 1000 == 0:
            print('error: ', error)
        count += 1

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

    if plot_error:
        import matplotlib
        matplotlib.use("TKAgg")
        import matplotlib.pyplot as plt
        plt.figure()
        plt.title("Trajectory Error")
        plt.plot(error_track)
        plt.ylabel("Distance to target [m]")
        plt.show()
