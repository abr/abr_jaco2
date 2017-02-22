"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position. Arm saves weights each run and
learns to adapt to unknown forces
"""
import numpy as np
import time
import os
import matplotlib.pyplot as plt
import gen_zeros as gen
import abr_control
import abr_jaco2

# ----TEST PARAMETERS-----
ii = 0 # need to manually count through trials as auto running does not work
name = 'adapt_ctrlr_test'
kp = 4.0
kv = 2.0
vmax = 0.1
num_runs = 2 # how many runs of learning to go through for averaging
num_trials = 3 # number of trials per run (cumulative learning)
regen = False # decide whether to regenerate functions for controller
save_history = 3  # number of latest weights files to save
save_data = True # whether to save data or not
plot_data = False # shows plot after run completes
time_limit = 10 # how long the arm is allowed to reach for the target [sec]
at_target = 200 # how long arm needs to be within tolerance of target [loop ctr]
num_targets = 1 # number of targets to move to in each trial

# parameters of adaptive controller
neural_backend = 'nengo'  # can be nengo, nengo_ocl, nengo_spinnaker
dim_in = 6 # number of dimensions
n_neurons = 10 # number of neurons (20k ~ max with 1 pop)
n_adapt_pop = 1 # number of adaptive populations
pes_learning_rate = 1.0e-7
voja_learning_rate = 1.0e-7
# ------------------------

count = 0 # loop counter
q_track = []
dq_track = []
ee_track = []
targets = []
error_track = []
target_index = 1
at_target_count = 0
# list of targets to move to
targets = [[-.4, .2, .70],
           [-.467, -.22, .78],
           [.467, -.22, .78],
           [.467, .22, .78],
           [-.467, .22, .78]]
target_xyz = targets[0]

# check if the weights file for n_neurons exists, if not create it
if not os.path.exists('data/learning_osc/%s/%i_neurons' % (name, n_neurons)):
    os.makedirs('data/learning_osc/%s/%i_neurons' % (name,n_neurons))

if (os.path.isfile('data/learning_osc/%s/%i_neurons/zeros.npz' % (name, n_neurons)) is False):
    gen.__init__(name, dim_in, n_neurons)

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config_neural(
    regenerate_functions=regen, use_cython=True,
    hand_attached=False)

# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.osc(
    robot_config, kp=kp, kv=kv, vmax=vmax)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(6), np.zeros(6), target_pos=np.zeros(3))

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)


for ii in range(0,num_trials):
    print('Run %i of %i' % (ii+1, num_trials))
    # connect to the jaco
    interface.connect()

    weights_location = []
    # load the weights files for each adaptive population
    if (ii == 0):
        print('Generating zeros weight files...')
        for jj in range(0, n_adapt_pop):
            weights_location.append('data/learning_osc/%s/%i_neurons/zeros.npz' % (name, n_neurons))
    else:
        print('Loading previous weights...')
        for jj in range(0, n_adapt_pop):
            weights_location.append('data/learning_osc/%s/%i_neurons/weights%i_pop%i.npz' %
                                    (name, n_neurons, (ii - 1) % save_history, jj))

    # instantiate the adaptive controller
    adapt = abr_control.controllers.signals.dynamics_adaptation(
        robot_config, backend=neural_backend,
        weights_file=weights_location,
        n_neurons=n_neurons,
        n_adapt_pop=n_adapt_pop,
        pes_learning_rate=pes_learning_rate,
        voja_learning_rate=voja_learning_rate)

    # run once to generate the functions we need
    adapt.generate(
        q=np.zeros(6), dq=np.zeros(6),
        training_signal=np.zeros(6))

    friction = abr_jaco2.signals.friction(robot_config)

    looptimes = np.zeros(num_trials)

    print('Moving to first target: ', target_xyz)

    try:
        # move to the home position
        interface.apply_q(robot_config.home_position_start)
        interface.init_force_mode()
        start_t = time.time()
        while 1:
            feedback = interface.get_feedback()
            q = np.array(feedback['q'])
            dq = np.array(feedback['dq'])
            hand_xyz = robot_config.Tx('EE', q=q)

            u = ctrlr.control(q=q, dq=dq, target_pos=target_xyz)
            u += adapt.generate(
                q=q, dq=dq,
                training_signal=ctrlr.training_signal)
            u += friction.generate(dq=dq)

            interface.send_forces(np.array(u, dtype='float32'))
            
            error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
            if error < .01:
                # if we're at the target, start count
                # down to moving to the next target
                at_target_count += 1
                if at_target_count >= at_target:
                    target_index += 1
                    if target_index > num_targets:
                        break
                    else:
                        target_xyz = targets[target_index]
                        print('Moving to next target: ', target_xyz)
                    at_target_count = 0
            if save_data is True:
                ee_track.append(hand_xyz)
                q_track.append(q)
                dq_track.append(dq)
            
            count += 1
            if count % 100 == 0:
                print('error: ', error)

            loop_time = time.time() - start_t

            if (loop_time) > time_limit:
                print("Time Limit Reached, Exiting...")
                break

    except Exception as e:
        print(e)

    finally:
        # return back to home position and close the connection to the arm
        interface.init_position_mode()
        interface.apply_q(robot_config.home_position_end)    
        interface.disconnect()

        if save_data is True:
            print('saving data...')
            # save weights from adaptive population
            for nn in range(0, n_adapt_pop):
                np.savez_compressed(
                    'data/learning_osc/%s/%i_neurons/weights%i_pop%i' %
                    (name, n_neurons, ii % save_history, nn),
                    weights=adapt.sim.data[adapt.probe_weights[nn]])

            np.savez_compressed('data/learning_osc/%s/%i_neurons/ee%i' % (name, 
                                n_neurons, ii), ee=ee_track)

            np.savez_compressed('data/learning_osc/%s/%i_neurons/q%i' % (name, 
                                n_neurons, ii), q=q_track)

            np.savez_compressed('data/learning_osc/%s/%i_neurons/dq%i' % (name, 
                                n_neurons, ii), dq=dq_track)

            
            # save time to target
            filename = ('data/learning_osc/%s/%i_neurons/total_time_track.txt' %
                        (name, n_neurons))
            if os.path.exists(filename):
                append_write = 'a' # append if file exists
            else:
                append_write = 'w' # make a new file if it does not exist
            time_file = open(filename, append_write)
            time_file.write('%.3f\n' % loop_time)
            time_file.close()
            print('Time to Target : %.3f' % loop_time)

            # save run info for plotting
            filename = ('data/learning_osc/read_info.txt')
            if os.path.exists(filename):
                append_write = 'a' # append if file exists
            else:
                append_write = 'w' # make a new file if it does not exist
            info_file = open(filename, append_write)
            info_file.seek(0)
            info_file.truncate()
            info_file.write('%s,%s,%s,%s' % (name, n_neurons, num_runs, num_trials))
            info_file.close()

         # plot last run
        """if plot_data is True:
            plt.subplot(2, 1, 1)
            plt.plot(q_track)
            plt.legend(range(6))
            plt.title('Joint angles')

            plt.subplot(2, 1, 2)
            plt.plot(dq_track[10:])
            plt.legend(range(6))
            plt.title('Joint velocities')

            plt.figure()
            plt.plot(error_track)
            plt.ylabel('Euclidean distance to target (m)')
            plt.xlabel('Time steps')
            plt.title('Error over time')

            ee_track = np.array(ee_track)
            targets_plot = np.ones(ee_track.shape) * target_xyz
            for ii in range(len(targets)):
                targets_plot[ii] = targets[ii]
            abr_control.utils.plotting.plot_trajectory(ee_track, targets_plot)

            plt.tight_layout()
            plt.show()"""