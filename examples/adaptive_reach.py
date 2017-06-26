"""
Move the jaco2 to a target position with an adaptive controller
that will account for a 2lb weight in its hand
"""
import numpy as np
import os
import timeit
import traceback

from abr_control.controllers import OSC, signals
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

# loop speed ~5ms, so 1ms in real time takes ~5ms
time_scale = 1/500

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, np.zeros(3))

robot_config.Tx('EE', q=zeros, x=robot_config.OFFSET)

interface = abr_jaco2.Interface(robot_config)

TARGET_XYZ = np.array([.57, .03, .87])

# weights_file = '~/.cache/abr_control/saved_weights/does_this_work/trial0/run9.npz'
weights_file = None
test_name = 'test1'
# create our adaptive controller
adapt = signals.DynamicsAdaptation(
    robot_config, backend='nengo_spinnaker',
    n_neurons=1000,
    n_adapt_pop=1,
    weights_file=weights_file,
    pes_learning_rate=1e-3,
    intercepts=(-0.1, 1.0),
    spiking=True,
    trial=None,
    run=None,
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
try:
    # interface.init_force_mode()
    # # get the end-effector's initial position
    feedback = interface.get_feedback()
    count = 0

    while 1:
        start = timeit.default_timer()
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()
        q = feedback['q']
        dq = feedback['dq']
        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=TARGET_XYZ)
        if u[0] > 0:
            u[0] *= 3.0
        else:
            u[0] *= 2.0
        u_adapt = adapt.generate(q=q, dq=dq,
                                 training_signal=ctrlr.training_signal)
        u += u_adapt * time_scale

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
        # interface.send_forces(np.array(u, dtype='float32'))
        #
        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=q)

        end = timeit.default_timer() - start
        # track data
        time_track.append(np.copy(end))
        q_track.append(np.copy(q))
        u_track.append(np.copy(u))
        adapt_track.append(np.copy(u_adapt))

        if count % 100 == 0:
            error = np.sqrt(np.sum((ee_xyz - TARGET_XYZ)**2))
            print('error: ', error)
            print('adapt: ', u_adapt)

        count += 1

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

    # Save the learned weights
    adapt.save_weights(test_name=test_name)
    # get save location of weights to save tracked data in same directory
    [location, run_num] = adapt.weights_location(test_name=test_name)

    print('Average loop speed: ', sum(time_track)/len(time_track))
    print('Run number ', run_num)
    print('Saving tracked data to ', location + '/run%i_data' % (run_num))

    time_track = np.array(time_track)
    q_track = np.array(q_track)
    u_track = np.array(u_track)
    adapt_track = np.array(adapt_track)
    TARGET_XYZ = np.array(TARGET_XYZ)

    if not os.path.exists(location + '/run%i_data' % (run_num)):
        os.makedirs(location + '/run%i_data' % (run_num))
    np.savez_compressed(location + '/run%i_data/q%i' % (run_num, run_num), q=[q_track])
    np.savez_compressed(location + '/run%i_data/time%i' % (run_num, run_num), time=[time_track])
    np.savez_compressed(location + '/run%i_data/u%i' % (run_num, run_num), u=[u_track])
    np.savez_compressed(location + '/run%i_data/adapt%i' % (run_num, run_num), adapt=[adapt_track])
    np.savez_compressed(location + '/run%i_data/target%i' % (run_num, run_num),
                        target=[TARGET_XYZ])

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
