"""
A class to use for the demo scripts. Handles the common code.

NOTE: To start the redis server run 'redis-server redis.conf' in the terminal
from the home directory
"""
import numpy as np
import redis

import abr_jaco2


class Demo(object):

    def __init__(self):
        self.count = 0
        self.move_home = False
        self.start_movement = False

        # create our interface for the jaco2
        self.interface = abr_jaco2.interface(self.robot_config)
        # connect to the jaco
        self.interface.connect()

        self.demo_init_torque_position = np.array(
            [0.0, 2.79, 2.62, 4.71, 0.0, 3.14], dtype="float32")

        # move to the home position
        self.interface.apply_q(self.demo_init_torque_position)

        self.demo_tooltip_read_positions = np.array(
            [[4.56668477, 2.09768783, 1.87899504,
              5.13325312, 1.63491513, 2.75462523]], dtype='float32')

        self.demo_pos_xyz = np.array([.40, -.18, .85])

        self.demo_pos_q = np.array(
            [0.36, 2.19, 2.63, 4.69, 0.024, 3.16], dtype="float32")

        # for communicating with the vision system
        self.redis_server = None

        self.offset = None

    def run(self):
        print('Arm Ready')

        # set up key input tracker
        self.kb = abr_jaco2.KBHit()

        self.mode = ''
        self.count = 0
        while 1:

            if self.mode == 'start':
                self.start_loop()

            elif self.mode == 'move_home':
                self.interface.apply_q(self.demo_init_torque_position)
                self.mode = ''

            elif self.mode == 'get_tooltip':
                self.get_tooltip_loop()

            if self.kb.kbhit():
                c = self.kb.getch()
                if ord(c) == 112:  # letter p, closes hand
                    self.interface.open_hand(False)
                if ord(c) == 111:  # letter o, opens hand
                    self.interface.open_hand(True)
                if ord(c) == 115:  # letter s, starts movement
                    if self.mode != 'start':
                        self.start_setup()
                        self.mode = 'start'
                if ord(c) == 116:  # letter t, starts tooltip reading process
                    if self.mode != 'get_tooltip':
                        self.interface.init_position_mode()
                        self.mode = 'get_tooltip'
                if ord(c) == 104:  # letter h, move to home
                    # switch to position control mode
                    self.interface.init_position_mode()
                    self.mode = 'move_home'
                if ord(c) == 113:  # letter q, quits and goes to finally
                   print('Returning to home position')
                   break;

            self.count += 1

    def stop(self):
        # return back to home position
        self.interface.init_position_mode()
        self.interface.apply_q(self.demo_init_torque_position)
        # close the connection to the arm
        self.interface.disconnect()
        # set the terminal back to its initial state
        self.kb.set_normal_term()

    def get_qdq(self, offset=[0, 0, 0]):
        # get feedback
        feedback = self.interface.get_feedback()
        self.q = np.array(feedback['q'])
        self.dq = np.array(feedback['dq'])

    def get_target_from_camera(self):
        # if not connected to server, connect and set up variables
        if self.redis_server is None:
            # create a server for the vision system to connect to
            self.redis_server = redis.StrictRedis(host='localhost')
            # self.target_xyz = self.robot_config.Tx(
            #     'EE', self.interface.get_feedback()['q'])

        # read from server
        camera_xyz = self.redis_server.get('target_xyz')
        # if the target has changed, recalculate things
        vals = str(camera_xyz)[2:-1].split(',')
        camera_xyz = np.array([float(val) for val in vals])
        # transform from camera to robot reference frame
        target_xyz = self.robot_config.Tx(
            'camera', x=camera_xyz, q=np.zeros(6))

        self.redis_server.set('transformed', '%g,%g,%g' % tuple(target_xyz))

        return target_xyz

    def normalize_target(self, target, magnitude=0.9):
        # set it so that target is not too far from joint 1
        joint1_offset = np.array([0, 0, 0.273])
        norm = np.linalg.norm(target - joint1_offset)
        if norm > magnitude:
            target = ((target - joint1_offset) / norm) * magnitude + joint1_offset
        self.redis_server.set('normalized', '%g,%g,%g' % tuple(target))
        return target

    def start_setup(self):
        raise Exception('start setup method not implemented')

    def start_loop(self):
        raise Exception('start loop method not implemented')

    def print_error(self, xyz, target):
        """ Print out the distance from the end-effector to xyz target """
        error = np.sqrt(np.sum((xyz - target)**2))
        if self.redis_server is not None:
            self.redis_server.set('error', error)  # Send to redis
            self.redis_server.set('xyz', xyz)
            self.redis_server.set(
                'training_signal', self.ctrlr.training_signal)
        print('error: ', error)

    def write_data(self):
        """ Write the data stored in the data dictionary to file """
        for key in self.tracked_data:
            np.savez_compressed('data/%s' % key, self.tracked_data[key])
