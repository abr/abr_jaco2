"""
A class to use for the demo scripts. Handles the common code.

NOTE: To start the redis server run 'redis-server redis.conf' in the terminal
from the home directory
"""
import numpy as np
import redis
import struct
import time

import abr_jaco2


class Demo(object):

    def __init__(self, track_data=False):

        self.track_data = track_data

        self.count = 0
        self.move_home = False
        self.start_movement = False
        self.trial = 0

        # create our interface for the jaco2
        self.interface = abr_jaco2.interface(self.robot_config)
        # connect to the jaco
        self.interface.connect()

        self.demo_init_torque_position = np.array(
            [0.0, 2.79, 2.62, 4.71, 0.0, 3.14], dtype="float32")

        self.demo_init_torque_xyz = np.array([0.0,0.0,1.20])

        # move to the home position
        self.interface.apply_q(self.demo_init_torque_position)

        self.demo_tooltip_read_positions = np.array(
            #[[4.83645727, 2.1057941, 1.95758253,
            #  2.74116295, 4.61607869, 3.46826159],
            [[5.52770083, 1.92617041, 1.85878244,
              4.29060418, 1.9736527, 5.65000764],
            [6.1248166, 1.35746381, 1.28933629,
              1.01879334, 6.26423963, 2.91255281]], dtype='float32')

        self.demo_pos_xyz = np.array([.57, 0.03, .87])

        self.demo_pos_q = np.array(
            [6.10003809, 1.75889646, 2.18678349,
             4.72202057, 0.02348013, 3.15828481], dtype="float32")

        # for communicating with the vision system
        self.redis_server = redis.StrictRedis(host='localhost')

        # redis variable to allow outside scripts to stop arm
        # set to false in case it is not reset at start of script
        self.redis_server.set("stop_arm", "False")
        # set neural activites to zero in redis
        self.redis_server.set("spikes", struct.pack('%dI' % 25, *[0]*25))
        # read the target_xyz from redis or not
        self.get_target_from_vision = False

        self.offset = None

    def run(self):
        print('Arm Ready')

        # set up key input tracker
        self.kb = abr_jaco2.KBHit()

        self.mode = ''
        self.count = 0
        while 1:

            if (self.redis_server.get(
                "stop_arm").decode('ascii')
                == "True"):
                break

            if self.mode == 'start':
                self.start_loop()

            elif self.mode == 'move_home':
                self.redis_server.set(
                    'norm_target_xyz_robot_coords', '%.3f %.3f %.3f'
                    % tuple(self.demo_init_torque_xyz))
                self.interface.apply_q(self.demo_init_torque_position)
                print('Reached home position')
                self.mode = ''

            elif self.mode == 'get_tooltip':
                self.get_tooltip_loop()

            # elif self.mode == 'float':
            #     self.interface.init_position_mode()
            #     self.interface.apply_q(self.demo_init_torque_position)
            #     self.interface.init_force_mode()

            elif self.mode == 'quit':
                break

            self.get_input()


            self.count += 1

    def stop(self):
        # return back to home position
        self.redis_server.set(
            'norm_target_xyz_robot_coords', '%.3f %.3f %.3f'
            % tuple(self.demo_init_torque_xyz))
        self.interface.init_position_mode()
        self.interface.apply_q(self.demo_init_torque_position)
        # close the connection to the arm
        self.interface.disconnect()
        # set the terminal back to its initial state
        self.kb.set_normal_term()
        self.redis_server.set("get_target", "False")
        # write data to file if it was tracked
        self.write_data()

    def get_input(self):
        if self.kb.kbhit():
            c = self.kb.getch()
            if ord(c) == 112:  # letter p, closes hand
                self.interface.open_hand(False)

            if ord(c) == 111:  # letter o, opens hand
                self.interface.open_hand(True)

            if ord(c) == 115:  # letter s, starts movement
                if self.mode != 'start':
                    # tell vision system to track target, if connected
                    if self.get_target_from_vision is True:
                        self.redis_server.set("get_target", "True")
                        print('Waiting for vision system to load')
                        while 1:
                            if (self.redis_server.get(
                                "network_running").decode('ascii')
                                == "True"):
                                break
                            time.sleep(1)

                    self.start_setup()
                    self.mode = 'start'

            if ord(c) == 116:  # letter t, starts tooltip reading process
                if self.mode != 'get_tooltip':
                    self.interface.init_position_mode()
                    self.mode = 'get_tooltip'

            if ord(c) == 102: # letter f, starts floating controller
                #self.mode = 'float'
                pass

            if ord(c) == 104:  # letter h, move to home
                # switch to position control mode
                self.interface.init_position_mode()
                self.mode = 'move_home'
                # tell vision system to stop tracking target, if connected
                self.redis_server.set("get_target", "False")

            if ord(c) == 113:  # letter q, quits and goes to finally
                print('Returning to home position')
                self.redis_server.set("get_target", "False")
                self.mode = 'quit'


    def get_qdq(self, offset=[0, 0, 0]):
        # get feedback
        feedback = self.interface.get_feedback()
        self.q = np.array(feedback['q'])
        self.dq = np.array(feedback['dq'])
        self.redis_server.set('q', '%.3f %.3f %.3f %.3f %.3f %.3f' %
                             (self.q[0],self.q[1],self.q[2],
                              self.q[3],self.q[4],self.q[5]))

    def get_target_from_camera(self):
        # read from server
        camera_xyz = self.redis_server.get('target_xyz').decode('ascii')
        # if the target has changed, recalculate things
        camera_xyz = np.array([float(val) for val in camera_xyz.split()])
        # transform from camera to robot reference frame
        target_xyz = self.robot_config.Tx(
            'camera', x=camera_xyz, q=np.zeros(6))

        self.redis_server.set(
            'target_xyz_robot_coords', '%.3f %.3f %.3f' % tuple(target_xyz))

        return target_xyz

    def normalize_target(self, target, magnitude=0.9):
        # set it so that target is not too far from joint 1
        joint1_offset = np.array([0, 0, 0.273])
        norm = np.linalg.norm(target - joint1_offset)
        if norm > magnitude:
            target = ((target - joint1_offset) / norm) * magnitude + joint1_offset
        # format used in nengo display, not sure if needs %g format
        #self.redis_server.set('normalized', '%g,%g,%g' % tuple(target))
        #self.redis_server.set(
        #    'norm_target_xyz_robot_coords', '%.3f %.3f %.3f' % (target[0],
        #    target[1], target[2]))
        return target

    def start_setup(self):
        raise Exception('start setup method not implemented')

    def start_loop(self):
        raise Exception('start loop method not implemented')

    def get_tooltip_loop(self):
        raise Exception('get tooltip loop method not implemented')

    def print_error(self, xyz, target):
        """ Print out the distance from the end-effector to xyz target """
        error = np.sqrt(np.sum((xyz - target)**2))
        self.redis_server.set('error', error)  # Send to redis
        self.redis_server.set('xyz', xyz)
        self.redis_server.set(
            'training_signal', self.ctrlr.training_signal)
        print('error: ', error)
        return error

    def write_data(self):
        """ Write the data stored in the data dictionary to file """
        if self.track_data is True:
            for key in self.tracked_data:
                np.savez_compressed('data/%s%i' % (key, self.trial),
                                    self.tracked_data[key])
