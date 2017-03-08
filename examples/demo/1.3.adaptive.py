"""
Demo script, adaptive hold position.
"""
import numpy as np
import redis
import timeit
import traceback

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
        self.offset = np.array([0.0, 0.0, 0.12])  # 20 cm from wrist

        # instantiate operation space controller
        self.ctrlr = abr_control.controllers.osc(
            self.robot_config, kp=20, kv=4, vmax=1, null_control=False)
        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        zeros = np.zeros(self.robot_config.num_joints)
        self.ctrlr.control(zeros, zeros, np.zeros(3))

        # instantiate the adaptive controller
        self.n_neurons = 2000
        self.adapt = abr_control.controllers.signals.dynamics_adaptation(
            self.robot_config, backend='nengo',
            n_neurons=self.n_neurons,
            n_adapt_pop=1,
            weights_file=weights_file,
            pes_learning_rate=1e-5,
            intercepts=(-0.1, 1.0),
            use_area_intercepts=True,
            spiking=False,
            extra_dimension=False,
            use_probes=True)

        # run once to generate the functions we need
        self.adapt.generate(zeros, zeros, zeros)

        # track data
        self.tracked_data = {'q': [], 'dq': []}

        # create a server for the vision system to connect to
        self.redis_server = redis.StrictRedis(host='localhost')
        self.redis_server.set("controller_name", "Adaptive")
        self.camera_xyz = '0, 0, 0'
        self.target_xyz = self.robot_config.Tx(
            'EE', self.interface.get_feedback()['q'])

    def start_setup(self):
        self.get_qdq()
        self.filtered_target = self.robot_config.Tx(
            'EE', q=self.q, x=self.offset)
        # switch to torque control mode
        self.interface.init_force_mode()
    previous = None
    def start_loop(self):
        # get position feedback from robot
        now = timeit.default_timer()
        if self.previous is not None and self.count%1000 == 0:
            print("dt:",now-self.previous)
            #Determine how many neurons are active then delete the data
            # if self.adapt.ens_activity is not None:
            #     if len(self.adapt.sim._probe_outputs[self.adapt.ens_activity]) != 0:
            #         tmp = self.adapt.sim._probe_outputs[self.adapt.ens_activity][-1]
            #         print("percent neurons active:", np.count_nonzero(tmp)/self.n_neurons)
            #         del self.adapt.sim._probe_outputs[self.adapt.ens_activity][:]

        self.previous = now
        self.get_qdq()
        self.filtered_target += .005 * (self.demo_pos_xyz - self.filtered_target)
        xyz = self.robot_config.Tx('EE', q=self.q, x=self.offset)

        # normalized target and incorporate offset
        self.target_subtraction(
            self.demo_pos_xyz, self.offset)

        # generate osc signal
        u = self.ctrlr.control(q=self.q, dq=self.dq,
                               target_pos=self.filtered_target)
        # generate adaptive signal
        adaptive = self.adapt.generate(
            q=self.q, dq=self.dq, training_signal=self.ctrlr.training_signal)
        u += adaptive

        # send control signal to Jaco 2
        self.interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if self.count % 100 == 0:
            self.print_error(xyz, self.demo_pos_xyz)

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
     print(traceback.format_exc())

finally:
    demo.stop()
    demo.write_data()
    # write weights from dynamics adaptation to file
    if demo.adapt.probe_weights is not None:
        np.savez_compressed(
            'data/weights_trial%i' % trial,
            weights=[demo.adapt.sim.data[demo.adapt.probe_weights[0]]])
