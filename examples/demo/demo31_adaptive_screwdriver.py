"""
Demo script, adaptive hold position.
"""
import numpy as np
import time
import traceback

from demo22_adaptive_reach import Demo22


class Demo31(Demo22):
    def __init__(self, weights_file):

        super(Demo22, self).__init__(
            weights_file, learning_rate=2e-5, use_probes=True)

        # account for wrist to tooltip offset
        self.offset = self.redis_server.get("offset")
        if self.offset is None:
            self.offset = np.array([0, 0, 0.12])
        else:
            self.offset = self.offset.decode('ascii')
            self.offset = np.array(
                [float(v) for v in self.offset.split()])

    def start_loop(self):
        super(Demo22, self).start_loop(
            magnitude=1.1, filter_const=0.0025)

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
                tooltip_camera = self.redis_server.get(
                    "tooltip").decode('ascii')
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

    demo = Demo31(weights_file)
    demo.run()

except Exception as e:
    print(traceback.format_exc())

finally:
    demo.stop()
    demo.write_data()
    # write weights from dynamics adaptation to file
    if demo.adapt.probe_weights is not None:
        print('Saving weights for trial %i' % trial)
        np.savez_compressed(
            'data/demo31_weights_trial%i' % trial,
            weights=[demo.adapt.sim.data[demo.adapt.probe_weights[0]]])
