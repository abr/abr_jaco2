"""
Move the jaco2 to a target position with an adaptive controller
that will account for a 2lb weight in its hand
"""
import numpy as np
import os
import timeit
import traceback
import redis

from abr_control.controllers import OSC, signals, path_planners
import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)

# get Jacobians to each link for calculating perturbation
J_links = [robot_config._calc_J('link%s' % ii, x=[0, 0, 0])
           for ii in range(robot_config.N_LINKS)]

# account for wrist to fingers offset
R_func = robot_config._calc_R('EE')

# instantiate controller
ctrlr = OSC(robot_config, kp=20, kv=6, vmax=1, null_control=True)
# ctrlr = Sliding(robot_config, kd=0.5, lamb=0.03)
path = path_planners.SecondOrder(robot_config)
n_timesteps = 4000

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, np.zeros(3))

robot_config.Tx('EE', q=zeros, x=robot_config.OFFSET)

interface = abr_jaco2.Interface(robot_config)

TARGET_XYZ = np.array([[.03, -.57, .87]]) #,
                       # [.42, .12, .62],
                       # [.32, .52, .75],
                       # [.15, .60, .87]])

time_limit = 10 # in seconds

weights_file = None
trial = None
run = None
test_name = 'adaptive_example'
# create our adaptive controller
adapt = signals.DynamicsAdaptation(
    n_input=6,
    n_output=3,
    n_neurons=1000,
    pes_learning_rate=1e-6,
    intercepts=(-0.1, 1.0),
    weights_file=weights_file,
    backend='nengo',
    trial=trial,
    run=run,
    test_name=test_name,
    autoload=False)

# connect to and initialize the arm
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

# set up lists for tracking data
time_track = []
q_track = []
u_track = []
adapt_track = []
error_track = []
training_track = []
try:
    w = 1e4/n_timesteps
    zeta = 2
    interface.init_force_mode()
    for ii in range(0,len(TARGET_XYZ)):
        print('Target %i/%i:' % (ii+1, len(TARGET_XYZ)), TARGET_XYZ[ii])

        # get the end-effector's initial position
        feedback = interface.get_feedback()
        count = 0
        loop_time = 0
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()
        q = feedback['q']
        dq = feedback['dq']
        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=q, x= robot_config.OFFSET)
        dt = 0.003

        target = np.concatenate((ee_xyz, np.array([0, 0, 0])), axis=0)

        while loop_time < time_limit:
            start = timeit.default_timer()
            prev_xyz = ee_xyz
            target = path.step(y=target[:3], dy=target[3:], target=TARGET_XYZ[ii], w=w,
                               zeta=zeta, dt = dt)
            # get joint angle and velocity feedback
            feedback = interface.get_feedback()
            q = feedback['q']
            dq = feedback['dq']
            # calculate end-effector position
            ee_xyz = robot_config.Tx('EE', q=q, x= robot_config.OFFSET)
            # calculate the control signal
            u_base = ctrlr.generate(
                q=q,
                dq=dq ,
                target_pos=target[:3],
                target_vel=target[3:],
                offset = robot_config.OFFSET)
            if u_base[0] > 0:
                u_base[0] *= 3.0
            else:
                u_base[0] *= 2.0
            training_signal = ctrlr.training_signal[:3]
            u_adapt = adapt.generate(
                        input_signal=np.concatenate((robot_config.scaledown('q',q)[:3],
                                                    robot_config.scaledown('dq',dq)[:3]),
                                                    axis=0),
                        training_signal=training_signal[:3])
            u = u_base + np.concatenate((u_adapt,
                                        np.array([0,0,0])), axis=0)


            # send forces into VREP, step the sim forward
            interface.send_forces(np.array(u, dtype='float32'))
            #
            error = np.sqrt(np.sum((ee_xyz - TARGET_XYZ[ii])**2))

            # track data
            q_track.append(np.copy(q))
            u_track.append(np.copy(u))
            adapt_track.append(np.copy(u_adapt))
            error_track.append(np.copy(error))
            training_track.append(np.copy(training_signal))
            end = timeit.default_timer() - start
            loop_time += end
            time_track.append(np.copy(end))

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

    # Save the learned weights
    adapt.save_weights(test_name=test_name, trial=trial, run=run)
    # get save location of weights to save tracked data in same directory
    [location, run_num] = adapt.weights_location(test_name=test_name, run=run,
                                                 trial=trial)
    print('Average loop speed: ', sum(time_track)/len(time_track))
    print('Run number ', run_num)
    print('Saving tracked data to ', location + '/run%i_data' % (run_num))

    time_track = np.array(time_track)
    q_track = np.array(q_track)
    u_track = np.array(u_track)
    adapt_track = np.array(adapt_track)
    error_track = np.array(error_track)
    training_track = np.array(training_track)

    if not os.path.exists(location + '/run%i_data' % (run_num)):
        os.makedirs(location + '/run%i_data' % (run_num))

    np.savez_compressed(location + '/run%i_data/q%i' % (run_num, run_num),
                        q=[q_track])
    np.savez_compressed(location + '/run%i_data/time%i' % (run_num, run_num),
                        time=[time_track])
    np.savez_compressed(location + '/run%i_data/u%i' % (run_num, run_num),
                        u=[u_track])
    np.savez_compressed(location + '/run%i_data/adapt%i' % (run_num, run_num),
                        adapt=[adapt_track])
    np.savez_compressed(location + '/run%i_data/target%i' % (run_num, run_num),
                        target=[TARGET_XYZ])
    np.savez_compressed(location + '/run%i_data/error%i' % (run_num, run_num),
                        error=[error_track])
    np.savez_compressed(location + '/run%i_data/training%i' % (run_num, run_num),
                        training=[training_track])
