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
import gen_zeros as gen
import abr_control
import abr_jaco2
import gc

# ----TEST PARAMETERS-----
calc_offset = True
s = 0 # have to manually go through runs
name = '2.2'
notes = 'starting from friction_training5'
kp = 4.0
kv = 2.0
vmax = 0.5
num_trials = 1  # how many trials of learning to go through for averaging
num_runs = 1  # number of runs per trial (cumulative learning)
save_history = 3   # number of latest weights files to save
save_data = False  # whether to save joint angle and vel data or not
save_learning = False  # whether the weights and plotting data get saved
time_limit = None  # Not used
at_target = 200  # how long arm needs to be within tolerance of target
num_targets = 1  # number of targets to move to in each trial

# parameters of adaptive controller
neural_backend = 'nengo'  # can be nengo, nengo_ocl, nengo_spinnaker
dim_in = 6  # number of dimensions
n_neurons = 20000  # number of neurons (20k ~ max with 1 pop)
n_adapt_pop = 1  # number of adaptive populations
pes_learning_rate = 1.0e-2
# ------------------------

count = 0  # loop counter
q_track = []
dq_track = []
ee_track = []
targets = []
error_track = []
target_index = 1
at_target_count = 0
# list of targets to move to
targets = [[.4, .2, .70],
           [.3, -.22, .60],
           [.45, 0.0, .65]]

target_xyz = targets[0]
# check if the weights file for n_neurons exists, if not create it
if not os.path.exists('data/learning_osc/%s/%i_neurons' % (name, n_neurons)):
    os.makedirs('data/learning_osc/%s/%i_neurons' % (name, n_neurons))

# initialize our robot config for neural controllers
#robot_config = abr_jaco2.robot_config_neural(
#    use_cython=True, hand_attached=False)
robot_config = abr_jaco2.robot_config_neural(
    use_cython=True, hand_attached=True)
robot_config.generate_control_functions(x=[1, 1, 1])

# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.osc(
    robot_config, kp=kp, kv=kv, vmax=vmax)

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)
# connect to the jaco
interface.connect()

read_positions = [robot_config.demo_tooltip_read_pos]
if calc_offset is True:
    # Move to read positions
    for ii in range(0, len(read_positions)):
        # move to read position ii
        print('Moving to read position ', ii)
        interface.apply_q(read_positions[ii])

        from abr_vision import robot_output, dataset, realsense
        cap = realsense.RealsenseFeed(
            (1080, 1920), depth_capture_shape=(240, 320))
        camera_xyz = robot_output.detect_tool(
            cap, find_first=True, crop_ratio=0.75)

        # calculate offset of tooltip from end-effector
        T_inv = robot_config.T_inv('EE', q=q)
        robot_config.offset = np.dot(T_inv, np.hstack([object_start, 1]))[:-1]

    # calculate average offset
    offset = offset_buf.mean(axis=0)
    print('Average Offset: ', offset)

else:
    offset = manual_offset

# # run controller once to generate functions / take care of overhead
# # outside of the main loop, because force mode auto-exits after 200ms
# #robot_config.generate_control_functions()
# ctrlr.control(np.zeros(6), np.zeros(6), target_pos=np.zeros(3))
# f = s+1
# for hh in range(0, num_trials):
#     for ii in range(s, f):
#         print('Run %i/%i in trial %i/%i' % (ii+1, num_runs, hh+1, num_trials))
#         if not os.path.exists('data/learning_osc/%s/%i_neurons/trial%i' %
#                               (name, n_neurons, hh)):
#             os.makedirs('data/learning_osc/%s/%i_neurons/trial%i' %
#                         (name, n_neurons, hh))
#
#         weights_location = []
#         # load the weights files for each adaptive population
#         if (ii == 0):
#             print('Loading friction comp weight files...')
#             for jj in range(0, n_adapt_pop):
#                 weights_location.append(
#                     'data/learning_osc/%s/%i_neurons/learned_baseline.npz' %
#                     (name, n_neurons))
#         else:
#             print('Loading previous weights...')
#             for jj in range(0, n_adapt_pop):
#                 weights_location.append(
#                     'data/learning_osc/' +
#                     '%s/%i_neurons/trial%i/weights%i_pop%i.npz' %
#                     (name, n_neurons, hh, (ii - 1) % save_history, jj))
#
#         # instantiate the adaptive controller
#         adapt = abr_control.controllers.signals.dynamics_adaptation(
#             robot_config, backend=neural_backend,
#             weights_file=weights_location,
#             n_neurons=n_neurons,
#             n_adapt_pop=n_adapt_pop,
#             pes_learning_rate=pes_learning_rate)
#
#         # run once to generate the functions we need
#         adapt.generate(
#             q=np.zeros(6), dq=np.zeros(6),
#             training_signal=np.zeros(6))
#
#         friction = abr_jaco2.signals.friction(robot_config)
#
#         looptimes = np.zeros(num_trials)
#
#         # connect to the jaco
#         interface.connect()
#
#         try:
#             kb = abr_jaco2.KBHit()
#             start_movement = False
#             # move to the home position
#             print('Moving to start position')
#             interface.apply_q(robot_config.home_position_start)
#             print('Moving to first target: ', target_xyz)
#             start_t = time.time()
#             avg_loop_time = 0.0
#             loop_time = time.time()
#             while 1:
#                 if start_movement:
#                     loop_start = time.time()
#                     feedback = interface.get_feedback()
#                     q = np.array(feedback['q'])
#                     dq = np.array(feedback['dq'])
#                     hand_xyz = robot_config.Tx('EE', q=q)
#
#                     u = ctrlr.control(q=q, dq=dq, target_pos=target_xyz)
#                     u += adapt.generate(
#                         q=q, dq=dq,
#                         training_signal=ctrlr.training_signal)
#
#                     interface.send_forces(np.array(u, dtype='float32'))
#
#                     error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
#
#                     ee_track.append(hand_xyz)
#
#                     if save_data is True:
#                         q_track.append(q)
#                         dq_track.append(dq)
#
#                     count += 1
#                     if count % 100 == 0:
#                         print('error: ', error)
#                         #print('ts: ', ctrlr.training_signal)
#
#                     avg_loop_time += time.time() - loop_start
#                     loop_time = time.time() - start_t
#
#
#                 if kb.kbhit():
#                     c = kb.getch()
#                     if ord(c) == 112: # letter p, closes hand
#                         interface.open_hand(False)
#                     if ord(c) == 111: # letter o, opens hand
#                         interface.open_hand(True)
#                     if ord(c) == 115: # letter s, starts movement
#                         start_movement = True
#                         # switch to torque control mode
#                         interface.init_force_mode()
#
#
#         except Exception as e:
#             print(e)
#
#         finally:
#             # return back to home position and close the connection to the arm
#             interface.init_position_mode()
#             interface.apply_q(robot_config.home_position_start)
#             interface.disconnect()
#             kb.set_normal_term()
#
#             print('Average Loop Time: ', avg_loop_time / count)
#
#             if save_learning is True:
#                 # save weights from adaptive population
#                 for nn in range(0, n_adapt_pop):
#                     print('saving weights...')
#                     np.savez_compressed(
#                         'data/learning_osc/%s/%i_neurons/trial%i/weights%i_pop%i' %
#                         (name, n_neurons, hh, ii % save_history, nn), weights=
#                         [adapt.sim.data[adapt.probe_weights[nn]][-1]])
#
#                     print('saving plotting data...')
#                     np.savez_compressed('data/learning_osc/%s/%i_neurons/'
#                                         'trial%i/ee%i' % (name, n_neurons,
#                                                           hh, ii), ee=ee_track)
#
#                     # save time to target
#                     filename = ('data/learning_osc/%s/%i_neurons/trial%i/'
#                                 'total_time_track.txt' %
#                                 (name, n_neurons, hh))
#                     if os.path.exists(filename):
#                         append_write = 'a'  # append if file exists
#                     else:
#                         append_write = 'w'  # make a new file if it does not exist
#                     time_file = open(filename, append_write)
#                     time_file.write('%.3f\n' % loop_time)
#                     time_file.close()
#                     print('Time to Target : %.3f' % loop_time)
#
#                     # save time to target
#                     filename = ('data/learning_osc/%s/%i_neurons/'
#                                 'run_parameters.txt' %
#                                 (name, n_neurons))
#                     if os.path.exists(filename):
#                         append_write = 'a'  # append if file exists
#                     else:
#                         append_write = 'w'  # make a new file if it does not exist
#                     parameter_file = open(filename, append_write)
#                     parameter_file.seek(0)
#                     parameter_file.truncate(0)
#                     parameter_file.write('name: %s\n' % name +
#                                          'kp: %.3f\n' % kp +
#                                          'kv: %.3f\n' % kv +
#                                          'vmax: %.3f\n' % vmax +
#                                          'time limit: %.3f\n' % time_limit +
#                                          'at target: %.3f\n' % at_target +
#                                          'neural backend: %s\n' % neural_backend +
#                                          'dim in: %.3f\n' % dim_in +
#                                          'n neurons: %.3f\n' % n_neurons +
#                                          'n adapt pop: %.3f\n' % n_adapt_pop +
#                                          'pes: %.3f\n' % pes_learning_rate +
#                                          'notes: %s' % notes)
#                     parameter_file.close()
#
#                     # save run info for plotting
#                     filename = ('data/learning_osc/read_info.txt')
#                     if os.path.exists(filename):
#                         append_write = 'a'  # append if file exists
#                     else:
#                         append_write = 'w'  # make a new file if it does not exist
#                     info_file = open(filename, append_write)
#                     info_file.seek(0)
#                     info_file.truncate()
#                     info_file.write('%s,%s,%s,%s' % (name, n_neurons, num_trials,
#                                                      num_runs))
#                     info_file.close()
#
#                     if save_data is True:
#                         print('saving extra data...')
#
#                         np.savez_compressed('data/learning_osc/%s/%i_neurons/'
#                                             'trial%i/q%i' % (name, n_neurons,
#                                                              hh, ii), q=q_track)
#
#                         np.savez_compressed('data/learning_osc/%s/%i_neurons/'
#                                             'trial%i/dq%i' % (name, n_neurons,
#                                                               hh, ii), dq=dq_track)
#             else:
#                 print('Save learning turned off...exiting')
#
#             adapt = None
#             gc.collect()
