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

# redis_server = redis.StrictRedis(host='localhost')
# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)

# redis_server.set('norm_target_xyz_robot_coords', '%.3f %.3f %.3f'
#                       % tuple(robot_config.INIT_TORQUE_POSITION))
# get Jacobians to each link for calculating perturbation
J_links = [robot_config._calc_J('link%s' % ii, x=[0, 0, 0])
           for ii in range(robot_config.N_LINKS)]

# account for wrist to fingers offset
R_func = robot_config._calc_R('EE')

# instantiate controller
ctrlr = OSC(robot_config, kp=20, kv=6, vmax=1, null_control=True)
#path = path_planners.SecondOrder(robot_config)
n_timesteps = 1000

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, np.zeros(3))

robot_config.Tx('EE', q=zeros, x=robot_config.OFFSET)

interface = abr_jaco2.Interface(robot_config)

TARGET_XYZ = np.array([[.57, .03, .87]]) # ,
                       # [.52, .22, .52],
                       # [.12, -.45, .70],
                       # [-.30, -.10, .79]])

time_limit = 10 # in seconds

# loop speed ~3ms, so 1ms in real time takes ~3ms
time_scale = 1/750

# weights_file = '~/.cache/abr_control/saved_weights/does_this_work/trial0/run9.npz'
weights_file = None
trial = 0
run = None
test_name = 'repeated/nengo_spinnaker'
# create our adaptive controller
adapt = signals.DynamicsAdaptation(
    n_input=robot_config.N_JOINTS,
    n_output=robot_config.N_JOINTS,
    n_neurons=1000,
    pes_learning_rate=1e-3,
    intercepts=(-0.1, 1.0),
    weights_file=weights_file,
    backend='nengo_spinnaker',
    trial=trial,
    run=run,
    test_name=test_name,
    autoload=True)

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
    for ii in range(0,len(TARGET_XYZ)):
        # # get the end-effector's initial position
        feedback = interface.get_feedback()
        count = 0
        loop_time = 0
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()
        q = feedback['q']
        dq = feedback['dq']
        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=q, x= robot_config.OFFSET)

        interface.init_force_mode()

        while loop_time < time_limit:
        #while count < n_timesteps:
            start = timeit.default_timer()
            target = [.57, .03, .87, 0.1, 0.1, 0.1]
            # target = path.step(y=q, dy=dq, target=TARGET_XYZ[ii], w=w,
            #                       zeta=zeta, dt = 0.003)
            # self.redis_server.set('norm_target_xyz_robot_coords', '%.3f %.3f %.3f'
            #                       % tuple(target[:3]))
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
                # target_vel=target[3:],
                offset = robot_config.OFFSET)
            if u_base[0] > 0:
                u_base[0] *= 3.0
            else:
                u_base[0] *= 2.0
            # ctrlr2.generate(q=q, dq=dq, target_pos=target[:3],
            #     offset=robot_config.OFFSET)
            u_adapt = adapt.generate(
                input_signal=robot_config.scaledown('q',q),
                training_signal=ctrlr.training_signal)
            u = u_base + (u_adapt * time_scale
                          * np.array([1,1,1,0.05,0.05,0.05]))
            #
            # add an additional force for the controller to adapt to if unable
            # to add mass to arm
            # fake_gravity = np.arra if unable
            # to add mass to army([[0, -4.0, 0, 0, 0, 0]]).T
            # g = np.zeros((robot_config.N_JOINTS, 1))
            # for ii in range(robot_config.N_LINKS):
            #     pars = tuple(feedback['q']) + tuple([0, 0, 0])
            #     g += np.dot(J_links[ii](*pars).T, fake_gravity)
            # u += g.squeeze()


            # send forces into VREP, step the sim forward
            interface.send_forces(np.array(u, dtype='float32'))
            #
            error = np.sqrt(np.sum((ee_xyz - target[:3])**2))

            # track data
            q_track.append(np.copy(q))
            u_track.append(np.copy(u))
            adapt_track.append(np.copy(u_adapt))
            error_track.append(np.copy(error))
            training_track.append(np.copy(ctrlr.training_signal))
            end = timeit.default_timer() - start
            loop_time += end
            time_track.append(np.copy(end))

            if count % 1000 == 0:
                print('error: ', error)
                print('adapt: ', u_adapt*time_scale)
                # print('u: ', u_base)

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

    # ee_track = np.array(ee_track)
    # target_track = np.array(target_track)
    #
    # if ee_track.shape[0] > 0:
    #     # plot distance from target and 3D trajectory
    #     import matplotlib
    #     matplotlib.use("TKAgg")
    #     import matplotlib.pyplot as plt
    #     from abr_control.utils.plotting import plot_3D
    #
    #     plt.figure()
    #     plt.plot(np.sqrt(np.sum((np.array(target_track) -
    #                              np.array(ee_track))**2, axis=1)))
    #     plt.ylabel('Distance (m)')
    #     plt.xlabel('Time (ms)')
    #     plt.title('Distance to target')
    #
    #     plot_3D(ee_track, target_track)
    #     plt.show()
