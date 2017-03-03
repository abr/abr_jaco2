"""
Demo script, adaptive hold position.
"""
import numpy as np
import redis

import abr_control
import abr_jaco2
from demo_class import Demo

class Demo22(Demo):
    def __init__(self, weights_file):

        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config_neural_1_3(
            use_cython=True, hand_attached=True)

        super(Demo22, self).__init__()

        # account for wrist to fingers offset
        self.R_func = self.robot_config._calc_R('EE')
        self.fingers_offset = np.array([0.0, 0.0, 0.0])  # 20 cm from wrist

        # instantiate operation space controller
        self.ctrlr = abr_control.controllers.osc(
            self.robot_config, kp=10, kv=3, vmax=1, null_control=False)
        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        zeros = np.zeros(self.robot_config.num_joints)
        self.ctrlr.control(zeros, zeros, np.zeros(3))

        # instantiate the adaptive controller
        self.adapt = abr_control.controllers.signals.dynamics_adaptation(
            self.robot_config, backend='nengo',
            n_neurons=20000, n_adapt_pop=1,
            weights_file=weights_file,
            pes_learning_rate=1e-1, intercepts=(0.7, 1.0))
        # run once to generate the functions we need
        self.adapt.generate(zeros, zeros, zeros)

        # track data
        self.tracked_data = {'q': [], 'dq': []}

        # create a server for the vision system to connect to
        self.redis_server = redis.StrictRedis(host='localhost')
        self.camera_xyz = '0, 0, 0'
        self.target_xyz = self.robot_config.Tx(
            'EE', self.interface.get_feedback()['q'])

    def start_setup(self):
        # switch to torque control mode
        self.interface.init_force_mode()

    def start_loop(self):
        # get position feedback from robot
        self.get_qdq()

        # read from vision, update target if new
        # which also does the offset and normalization
        self.get_target_from_camera()

        # generate osc signal
        u = self.ctrlr.control(q=self.q, dq=self.dq,
                               target_pos=self.target_xyz)
        # generate adaptive signal
        adaptive = self.adapt.generate(
            q=self.q, dq=self.dq, training_signal=self.ctrlr.training_signal)
        u += adaptive

        # send control signal to Jaco 2
        self.interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if self.count % 100 == 0:
            self.print_error()

        # track data
        self.tracked_data['q'].append(np.copy(self.q))
        self.tracked_data['dq'].append(np.copy(self.dq))

try:

    # if trial = 0 it creates a new set of decoders = 0
    # otherwise it loads the weights from trial - 1
    trial = 0
    if trial > 0:
        weights_file = ['data/weights_trial%i.npz' % (trial - 1)]
    elif trial == 0:
        weights_file = None

    demo = Demo22(weights_file)
    demo.run()

except Exception as e:
     print(e)

finally:
    demo.stop()
    demo.write_data()
    # write weights from dynamics adaptation to file
    np.savez_compressed(
        'data/weights_trial%i' % trial,
        weights=[demo.adapt.sim.data[demo.adapt.probe_weights[0]]])
