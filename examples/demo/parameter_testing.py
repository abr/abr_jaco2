"""
Demo script, adaptive hold position.
"""
import numpy as np
import redis
import timeit
import traceback
import os

import abr_control
import abr_jaco2
from demo_class import Demo


class Demo22(Demo):
    def __init__(self, weights_file, track_data=True,
                 learning_rate=1e-5, use_probes=True):

        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config_neural(
            use_cython=True, hand_attached=True)

        super(Demo22, self).__init__(track_data)

        # create a server for the vision system to connect to
        # ------ CONTROL PARAMETERS --------
        self.kp = 20
        self.kv = 6
        self.vmax = 1
        self.null = False
        self.notes = 'One target, holding arm to build up force, no C or dJ'
        self.control = 'adaptive'

        self.learning_rate = learning_rate
        # ----------------------------------

        self.redis_server = redis.StrictRedis(host='localhost')
        self.redis_server.set("controller_name", "Adaptive")

        # account for wrist to fingers offset
        self.offset = np.array([0, 0, 0.12])

        # instantiate operation space controller
        self.ctrlr = abr_control.controllers.osc(
            self.robot_config, kp=self.kp, kv=self.kv, vmax=self.vmax,
            null_control=self.null)
        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        zeros = np.zeros(self.robot_config.num_joints)
        self.ctrlr.control(zeros, zeros, np.zeros(3), offset=self.offset)
        self.robot_config.Tx('EE', q=zeros, x=self.robot_config.offset)
        # self.robot_config.Tx('camera', x=np.ones(3), q=np.zeros(6))

        # instantiate the adaptive controller
        self.n_neurons = 10000
        self.adapt = abr_control.controllers.signals.dynamics_adaptation(
            self.robot_config, backend='nengo',
            n_neurons=self.n_neurons,
            n_adapt_pop=1,
            weights_file=weights_file,
            pes_learning_rate=learning_rate,
            intercepts=(-0.1, 1.0),
            use_area_intercepts=True,
            spiking=False,
            extra_dimension=False,
            use_probes=use_probes)

        # run once to generate the functions we need
        self.adapt.generate(zeros, zeros, zeros)

        # track data
        if self.track_data is True:
            self.tracked_data = {'q': [], 'dq': [], 'training_signal': [],
                                 'filtered_target': [], 'target': [], 'EE': [],
                                 'u': [], 'u_adapt': []}

        self.get_qdq()

        self.previous = None

        self.get_target_from_vision = True

    def start_setup(self):
        # switch to torque control mode
        self.interface.init_force_mode()

        # get position feedback from robot
        self.get_qdq()
        xyz = self.robot_config.Tx('EE', q=self.q, x=self.offset)
        self.filtered_target = xyz

    def start_loop(self, magnitude=.9, filter_const=0.005, white_noise=True):
        # get position feedback from robot
        now = timeit.default_timer()
        if self.previous is not None and self.count % 1000 == 0:
            print("dt:", now-self.previous)

        self.previous = now
        self.get_qdq()
        xyz = self.robot_config.Tx('EE', q=self.q, x=self.offset)

        # read from vision, update target if new
        # which also does the offset and normalization
        target_xyz = self.get_target_from_camera()
        target_xyz = self.normalize_target(target_xyz, magnitude=magnitude)
        # filter the target so that it doesn't jump, but moves smoothly
        self.filtered_target += filter_const * (
            target_xyz - self.filtered_target)

        if white_noise is True:
            center = 0
            #scale = 0.025
            scale = 0.001
            num_samples = 3
            noise = np.random.normal(center, scale, size=num_samples)

            self.filtered_target += noise

        self.redis_server.set(
            'norm_target_xyz_robot_coords', '%.3f %.3f %.3f'
            % (self.filtered_target[0],
            self.filtered_target[1],
            self.filtered_target[2]))

        # generate osc signal
        u = self.ctrlr.control(q=self.q, dq=self.dq,
                               target_pos=self.filtered_target,
                               offset=self.offset)
        u[0] *= 2.0
        # generate adaptive signal
        if self.control == 'adaptive':
            adaptive = self.adapt.generate(
                q=self.q, dq=self.dq, training_signal=self.ctrlr.training_signal)
            u += adaptive

        # send control signal to Jaco 2
        self.interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if self.count % 100 == 0:
            error = self.print_error(xyz, target_xyz, display_error=True)

        # track data
        if self.track_data is True:
            self.tracked_data['q'].append(np.copy(self.q))
            self.tracked_data['dq'].append(np.copy(self.dq))
            self.tracked_data['training_signal'].append(
                np.copy(self.ctrlr.training_signal))
            self.tracked_data['filtered_target'].append(
                np.copy(self.filtered_target))
            self.tracked_data['target'].append(np.copy(target_xyz))
            self.tracked_data['EE'].append(np.copy(xyz))
            self.tracked_data['u'].append(np.copy(u))
            if self.control == 'adaptive':
                self.tracked_data['u_adapt'].append(np.copy(adaptive))

        self.redis_server.set('u_adapt', '%.3f %.3f %.3f %.3f %.3f %.3f'
            % (adaptive[0], adaptive[1], adaptive[2],
               adaptive[3], adaptive[4], adaptive[5]))
try:

    # if trial = 0 it creates a new set of decoders = 0
    # otherwise it loads the weights from trial - 1
    trial = 0
    test_name = '/adaptive/training/noise_1e-3'
    data_folder = 'data/parameter_testing' + test_name
    abr_control.utils.os_utils.makedir(data_folder + '/trial%i/' % trial)
    if trial > 0:
        weights_file = ['data/parameter_testing/%s/trial%i/trial%i.npz' % (
          test_name, trial - 1, trial - 1)]
    elif trial == 0:
        weights_file = None

    demo22 = Demo22(weights_file, use_probes=True)
    demo22.data_folder = data_folder
    demo22.trial = trial
    demo22.run()

except Exception as e:
    print(traceback.format_exc())

finally:
    demo22.stop()
    # write weights from dynamics adaptation to filei
    if demo22.adapt.probe_weights is not None:
        print('Saving weights for trial %i' % demo22.trial)
        np.savez_compressed(
            '%s/trial%i/trial%i' % (
            demo22.data_folder, demo22.trial, demo22.trial),
            weights=[demo22.adapt.sim.data[demo22.adapt.probe_weights[0]]])

    # save time to target
    filename = ('%s/'
                'run_parameters.txt' %
                (demo22.data_folder))
    if os.path.exists(filename):
        append_write = 'a'  # append if file exists
    else:
        append_write = 'w'  # make a new file if it does not exist
    parameter_file = open(filename, append_write)
    parameter_file.seek(0)
    parameter_file.truncate(0)
    parameter_file.write('name: %s\n' % demo22.data_folder +
                         'kp: %.3f\n' % demo22.kp +
                         'kv: %.3f\n' % demo22.kv +
                         'vmax: %.3f\n' % demo22.vmax +
                         'null: %s\n' % demo22.null +
                         #'time limit: %.3f\n' % time_limit +
                         #'at target: %.3f\n' % at_target +
                         #'neural backend: %s\n' % neural_backend +
                         #'dim in: %.3f\n' % dim_in +
                         'controller: %s\n' % demo22.control +
                         'n neurons: %.3f\n' % demo22.n_neurons +
                         #'n adapt pop: %.3f\n' % n_adapt_pop +
                         'learning_rate: %f\n' % demo22.learning_rate +
                         'notes: %s' % demo22.notes)
