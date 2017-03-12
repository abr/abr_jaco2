"""
Demo script, adaptive hold position.
"""
import numpy as np
import redis
import time
import timeit
import traceback

import abr_control
import abr_jaco2
from demo_class import Demo


class Demo31(Demo):
    def __init__(self, weights_file):

        # initialize our robot config for neural controllers
        self.robot_config = abr_jaco2.robot_config_neural(
            use_cython=True, hand_attached=True)

        super(Demo31, self).__init__()

        # ------ CONTROL PARAMETERS --------
        kp = 20
        kv = 6
        vmax = 1
        null = True
        # ----------------------------------

        # create a server for the vision system to connect to
        # create a server for the vision system to connect to
        self.redis_server = redis.StrictRedis(host='localhost')
        self.redis_server.set("controller_name", "Compliant")

        # account for wrist to fingers offset
        self.offset = self.redis_server.get("offset")
        if self.offset is None:
            self.offset = np.array([0, 0, 0.12])
        else:
            self.offset = self.offset.decode('ascii')
            self.offset = np.array(
                [float(v) for v in self.offset.split()])

        # instantiate operation space controller
        self.ctrlr = abr_control.controllers.osc(
            self.robot_config, kp=kp, kv=kv, vmax=vmax, null_control=null)
        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        zeros = np.zeros(self.robot_config.num_joints)
        self.ctrlr.control(zeros, zeros, np.zeros(3), offset=self.offset)

        # track data
        if self.track_data is True:
            self.tracked_data = {'q': [], 'dq': [], 'training_signal': [],
                                 'filtered_target': [], 'target': [], 'EE': []}

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

    def start_loop(self, magnitude=1.0, filter_const=0.005):
        # get position feedback from robot
        now = timeit.default_timer()
        if self.previous is not None and self.count%1000 == 0:
            print("dt:",now-self.previous)
            #Determine how many neurons are active then delete the data
            #if len(self.adapt.sim._probe_outputs[self.adapt.ens_activity]) != 0:
            #    tmp = self.adapt.sim._probe_outputs[self.adapt.ens_activity][-1]
            #    print("percent neurons active:", np.count_nonzero(tmp)/self.n_neurons)
            #    del self.adapt.sim._probe_outputs[self.adapt.ens_activity][:]

        self.previous = now
        self.get_qdq()
        xyz = self.robot_config.Tx('EE', q=self.q, x=self.offset)

        # read from vision, update target if new
        # which also does the offset and normalization
        target_xyz = self.get_target_from_camera()
        target_xyz = self.normalize_target(target_xyz, magnitude=magnitude)
        # filter the target so that it doesn't jump, but moves smoothly
        self.filtered_target += filter_const * (target_xyz - self.filtered_target)
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

        # send control signal to Jaco 2
        self.interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if self.count % 100 == 0:
            self.print_error(xyz, target_xyz)
            print('current xyz: ', xyz)
            print('target_xyz: ', target_xyz)

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

    def get_tooltip_loop(self):
        num_positions = len(self.demo_tooltip_read_positions)
        tooltip_offsets = np.zeros((num_positions, 3))
        # Move to read positions
        for ii in range(num_positions):

            # move to read position ii
            print('Moving to read position ', ii)
            self.interface.apply_q(
                self.demo_tooltip_read_positions[ii])

            time.sleep(1)  # stabilize

            print('Waiting for feedback from vision system')
            # read tooltip position from camera
            self.redis_server.set("tooltip", "")
            self.redis_server.set("get_tooltip", "True")
            while self.mode == 'get_tooltip':
                tooltip_camera = self.redis_server.get("tooltip").decode('ascii')
                print(tooltip_camera)

                # check keyboard input
                self.get_input()

                if tooltip_camera != "":
                    self.redis_server.set("get_tooltip", "False")
                    break
                time.sleep(1)

            # Used to exit if user hits 'q' or 'h'
            # TODO: CLEAN THIS UP
            if self.mode != 'get_tooltip':
                return

            tooltip_camera = [float(v) for v in tooltip_camera.split()]
            # convert to robot coordinates
            tooltip_xyz = self.robot_config.Tx(
                'camera', x=tooltip_camera, q=np.zeros(6))

            # calculate offset of tooltip from end-effector
            self.get_qdq()
            T_inv = self.robot_config.T_inv('EE', q=self.q)
            tooltip_offsets[ii] = np.dot(
                T_inv, np.hstack([tooltip_xyz, 1]))[:-1]

        # calculate average offset
        self.offset = tooltip_offsets.mean(axis=0)
        self.redis_server.set(
            "offset", '%.3f %.3f %.3f' % tuple(self.offset))
        print('Estimated tooltip offset from end-effector: ', self.offset)

        # return to home position
        self.mode = 'move_home'

try:

    # if trial = 0 it creates a new set of decoders = 0
    # otherwise it loads the weights from trial - 1
    trial = 0
    if trial > 0:
        weights_file = ['data/demo31_weights_trial%i.npz' % (trial - 1)]
    elif trial == 0:
        weights_file = None

    demo31 = Demo31(weights_file)
    demo31.trial = trial
    demo31.run()

except Exception as e:
     print(traceback.format_exc())

finally:
    demo31.stop()
    # write weights from dynamics adaptation to file
