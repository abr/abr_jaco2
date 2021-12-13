"""
Move the jaco2 to a target position with an adaptive controller
that will account for unknown perturbations

The adaptive controller takes in the joint positions and velocities for the
second and third joints (shoulder, elbow) and outputs a control signal [Nm] for
the two respectively.
"""

import numpy as np
import os
import timeit
import traceback

try:
    import nengo
except ImportError as e:
    print("To run adaptive control, please 'pip install nengo' first")
    raise e

from abr_control.controllers import OSC, signals
from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
import abr_jaco2

plot_error = True

# initialize our robot config
robot_config = abr_jaco2.Config()

# instantiate our null controllers
null_controllers = []
from abr_control.controllers import Damping
null_controllers.append(Damping(
    robot_config=robot_config, kv=10))

# instantiate controller
ctrlr = OSC(robot_config, kp=30, kv=20, vmax=None,
            null_controllers=null_controllers)

# instantiate path planner and set parameters
Pprof = Linear()
Vprof = Gaussian(dt=0.001, acceleration=1)
path = PathPlanner(pos_profile=Pprof, vel_profile=Vprof, verbose=True)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)

robot_config.Tx('EE', q=zeros)

ctrlr.generate(zeros, zeros, zeros)

interface = abr_jaco2.Interface(robot_config)

target_xyz = np.array([.57, 0.03, .87])

time_limit = 30 # in seconds

# create our adaptive controller
adapt = signals.DynamicsAdaptation(
    n_input=4,
    n_output=2,
    n_neurons=1000,
    pes_learning_rate=1e-4,
    means=[3.14, 3.14, 0.05, 0.05],
    variances=[3.14, 3.14, 1, 1],
    weights=None)

# connect to and initialize the arm
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.START_ANGLES)

if plot_error:
    error_track = []

try:
    # get the end-effector's initial position
    feedback = interface.get_feedback()
    ee_xyz = robot_config.Tx('EE', q=feedback['q'])
    count = 0
    loop_time = 0

    path.generate_path(
        start_position=ee_xyz,
        target_position=target_xyz,
        max_velocity=2,
        start_velocity=0,
        target_velocity=0,
    )

    interface.init_force_mode()

    # get joint angle and velocity feedback
    feedback = interface.get_feedback()
    # calculate end-effector position
    position = robot_config.Tx('EE', q=feedback['q'])
    velocity = np.array([0, 0, 0])

    while loop_time < time_limit:
        start = timeit.default_timer()
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])

        # get next step along trajectory
        filtered_target = path.step()

        feedback = interface.get_feedback()

        # calculate the control signal
        u_base = ctrlr.generate(
            q=feedback['q'], dq=feedback['dq'],
            target=np.hstack((filtered_target[:3], [0, 0, 0])),
            target_velocity=np.hstack((filtered_target[3:6], [0, 0, 0])),
            ref_frame='EE')

        # calculate the adaptive control signal
        input_signal = np.hstack((feedback['q'][1:3], feedback['dq'][1:3]))
        training_signal = np.array([ctrlr.training_signal[1],
                                    ctrlr.training_signal[2]])
        u_adapt = adapt.generate(
            input_signal=input_signal, training_signal=training_signal)

        u_adapt = np.array([0, u_adapt[0], u_adapt[1], 0, 0, 0])
        u = u_base + u_adapt

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
    interface.send_target_angles(robot_config.START_ANGLES)
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
