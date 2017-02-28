"""
1 basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position. Arm saves weights each run and
learns to adapt to unknown forces
"""
import numpy as np
import time
import abr_control
import abr_jaco2
import os

kp = 10.0
kv = 3.0
vmax = 1.0

# ----TEST PARAMETERS-----
run_index = 13  # have to manually go through runs
name = '2.2.0.8'
notes = ''
num_trials = 1  # how many trials of learning to go through for averaging
num_runs = 30  # number of runs per trial (cumulative learning)
save_history = 3   # number of latest weights files to save
save_data = True  # whether to save joint angle and vel data or not
# TO DO: save data does not save if save learning is false
# TODO: if save_learning is false data doesn't save, separate
save_learning = True  # whether the weights and plotting data get saved
time_limit = 1000  # Not used
at_target = 200  # how long arm needs to be within tolerance of target
start_time = 0

# parameters of adaptive controller
neural_backend = 'nengo'  # can be nengo, nengo_ocl, nengo_spinnaker
dim_in = 12  # number of dimensions
n_neurons = 20000  # number of neurons (20k ~ max with 1 pop)
n_adapt_pop = 1  # number of adaptive populations
pes_learning_rate = 1.5e-1
intercepts = (0.7, 1.0)
# ------------------------

count = 0  # loop counter
q_track = []
dq_track = []
adaptive_track = []
ee_track = []

# filename where vision system writes object xyz coordinates
filename = 'data/target_position.txt'

# check if the weights file for n_neurons exists, if not create it
abr_control.utils.os_utils.makedir(
    'data/learning_osc/%s/%i_neurons' % (name, n_neurons))

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config_neural_2_2(
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

f = run_index + 1
for hh in range(0, num_trials):
    for ii in range(run_index, f):
        print('Run %i/%i in trial %i/%i' % (ii+1, num_runs, hh+1, num_trials))
        abr_control.utils.os_utils.makedir(
            'data/learning_osc/%s/%i_neurons/trial%i' % (name, n_neurons, hh))

        weights_location = []
        # load the weights files for each adaptive population
        for jj in range(0, n_adapt_pop):
            weights_location.append(
                'data/learning_osc/' +
                '%s/%i_neurons/trial%i/weights%i_pop%i.npz' %
                (name, n_neurons, hh, (ii - 1) % save_history, jj))

        # instantiate the adaptive controller
        adapt = abr_control.controllers.signals.dynamics_adaptation(
            robot_config, backend=neural_backend,
            weights_file=weights_location,
            n_neurons=n_neurons,
            n_adapt_pop=n_adapt_pop,
            pes_learning_rate=pes_learning_rate,
            intercepts=intercepts)

        # run once to generate the functions we need
        # TODO: double check what the full training signal should be
        adapt.generate(q=np.zeros(6), dq=np.zeros(6),
                       training_signal=np.zeros(6))

        looptimes = np.zeros(num_trials)

        # connect to the jaco
        interface.connect()

        move_home = False
        start_movement = False
        try:
            # start key input tracker
            kb = abr_jaco2.KBHit()

            # move to the home position
            print('Moving to start position')
            interface.apply_q(robot_config.home_position_start)
            move_home = False

            print('Arm ready')

            while 1:

                if start_movement:
                    # get robot feedback
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
                            # set it so that target is no farther than .8m away
                            norm = np.linalg.norm(target_xyz)
                            if norm > .8:
                                target_xyz = (target_xyz /
                                              np.linalg.norm(target_xyz) * .8)

                        # calculate and print error
                        hand_xyz = robot_config.Tx('EE', q=q)
                        error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
                        print('error: ', error)

                    if target_xyz is None:
                        target_xyz = robot_config.Tx('EE', q=q)

                    # generate osc signal
                    # TODO NOTE: when it uses offset shit is terrible, WHY???
                    # NOTE: maybe just change target position by offset amount for now
                    u = ctrlr.control(q=q, dq=dq, target_pos=target_xyz)
                                      # offset=[0, 0, .001])
                    # generate adaptive dynamics signal
                    adaptive = adapt.generate(
                        q=q, dq=dq, training_signal=ctrlr.training_signal)
                    u += adaptive

                    # send control signal to robot arm
                    interface.send_forces(np.array(u, dtype='float32'))

                    if save_data is True:
                        q_track.append(q)
                        dq_track.append(dq)
                        adaptive_track.append(adaptive)

                    count += 1

                    if time.time() - start_time > time_limit:
                        print('Time limit reach')
                        break;

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
                        start_time = time.time()
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
            # return back to home position and close the connection to the arm
            interface.init_position_mode()
            interface.apply_q(robot_config.home_position_start)
            interface.disconnect()
            kb.set_normal_term()

            if save_learning is True:
                # save weights from adaptive population
                for nn in range(0, n_adapt_pop):
                    print('saving weights...')
                    np.savez_compressed(
                        'data/learning_osc/' +
                        '%s/%i_neurons/trial%i/weights%i_pop%i' %
                        (name, n_neurons, hh, ii % save_history, nn),
                        weights=[adapt.sim.data[adapt.probe_weights[nn]][-1]])

                    print('saving plotting data...')
                    np.savez_compressed('data/learning_osc/%s/%i_neurons/'
                                        'trial%i/ee%i' % (name, n_neurons,
                                                          hh, ii), ee=ee_track)

                    # save time to target
                    filename = ('data/learning_osc/%s/%i_neurons/trial%i/'
                                'total_time_track.txt' %
                                (name, n_neurons, hh))
                    if os.path.exists(filename):
                        append_write = 'a'  # append if file exists
                    else:
                        append_write = 'w'  # make file if it does not exist

                    # save parameters list
                    filename = ('data/learning_osc/%s/%i_neurons/'
                                'run_parameters.txt' %
                                (name, n_neurons))
                    if os.path.exists(filename):
                        append_write = 'a'  # append if file exists
                    else:
                        append_write = 'w'  # make file if it does not exist
                    parameter_file = open(filename, append_write)
                    parameter_file.seek(0)
                    parameter_file.truncate(0)
                    parameter_file.write(
                        'name: %s\n' % name +
                        'kp: %.3f\n' % kp +
                        'kv: %.3f\n' % kv +
                        'vmax: %.3f\n' % vmax +
                        'time limit: %.3f\n' % time_limit +
                        'at target: %.3f\n' % at_target +
                        'neural backend: %s\n' % neural_backend +
                        'dim in: %.3f\n' % dim_in +
                        'n neurons: %.3f\n' % n_neurons +
                        'n adapt pop: %.3f\n' % n_adapt_pop +
                        'pes: %.3f\n' % pes_learning_rate +
                        'notes: %s' % notes)
                    parameter_file.close()

                    # save run info for plotting
                    filename = ('data/learning_osc/read_info.txt')
                    if os.path.exists(filename):
                        append_write = 'a'  # append if file exists
                    else:
                        append_write = 'w'  # make a new file if DNE
                    info_file = open(filename, append_write)
                    info_file.seek(0)
                    info_file.truncate()
                    info_file.write('%s,%s,%s,%s' %
                                    (name, n_neurons, num_trials, num_runs))
                    info_file.close()

                    if save_data is True:
                        print('saving extra data...')

                        np.savez_compressed(
                            'data/learning_osc/%s/%i_neurons/'
                            'trial%i/q%i' % (name, n_neurons, hh, ii),
                            q=q_track)

                        np.savez_compressed(
                            'data/learning_osc/%s/%i_neurons/'
                            'trial%i/dq%i' % (name, n_neurons, hh, ii),
                            dq=dq_track)

                        np.savez_compressed(
                            'data/learning_osc/%s/%i_neurons/'
                            'trial%i/adapt%i' % (name, n_neurons, hh, ii),
                            adapt=adaptive_track)
            else:
                print('save_learning is False...exiting')
