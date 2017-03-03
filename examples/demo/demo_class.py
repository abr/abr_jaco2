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
        # move to the home position
        self.interface.apply_q(self.robot_config.init_torque_position)

        self.demo_tooltip_read_pos = np.array(
            [1.80, 3.26, 2.60, 1.04, 2.26, 1.65], dtype='float32')

        self.demo_pos_xyz = np.array([.40, -.18, .85])

        self.demo_pos_q = np.array(
            [0.36, 2.19, 2.63, 4.69, 0.024, 3.16], dtype="float32")

        # for communicating with the vision system
        self.redis_server = None

    def run(self):
        print('Arm Ready')

        # set up key input tracker
        self.kb = abr_jaco2.KBHit()

        self.count = 0
        while 1:

            if self.start_movement is True:
                self.start_loop()

            if self.move_home is True:
                self.interface.init_position_mode()
                self.interface.apply_q(self.robot_config.init_torque_position)
                self.move_home = False

            if self.kb.kbhit():
                c = self.kb.getch()
                if ord(c) == 112:  # letter p, closes hand
                    self.interface.open_hand(False)
                if ord(c) == 111:  # letter o, opens hand
                    self.interface.open_hand(True)
                if ord(c) == 115:  # letter s, starts movement
                    self.start_setup()
                    self.start_movement = True
                if ord(c) == 104:  # letter h, move to home
                    self.start_movement = False
                    self.move_home = True
                    # switch to position control mode
                    self.interface.init_position_mode()
                if ord(c) == 113:  # letter q, quits and goes to finally
                   print('Returning to home position')
                   break;

            self.count += 1

    def stop(self):
        # return back to home position
        self.interface.init_position_mode()
        self.interface.apply_q(self.robot_config.init_torque_position)
        # close the connection to the arm
        self.interface.disconnect()
        self.kb.set_normal_term()

    def get_qdq(self):
        # get feedback
        feedback = self.interface.get_feedback()
        self.q = np.array(feedback['q'])
        self.dq = np.array(feedback['dq'])

    def get_target_from_camera(self):
        # if not connected to server, connect and set up variables
        if self.redis_server is None:
            self.redis_server = redis.StrictRedis(host='localhost')
            # create a server for the vision system to connect to
            self.camera_xyz = '0, 0, 0'
            self.target_xyz = self.robot_config.Tx(
                'EE', self.interface.get_feedback()['q'])

        # read from server
        new_camera_xyz = self.redis_server.get('target_xyz')
        # if the target has changed, recalculate things
        if not self.camera_xyz == new_camera_xyz:
            self.camera_xyz = np.copy(new_camera_xyz)
            vals = str(self.camera_xyz)[2:-1].split(',')
            camera_xyz = np.array([float(val) for val in vals])
            # transform from camera to robot reference frame
            self.target_xyz = self.robot_config.Tx(
                'camera', x=camera_xyz, q=np.zeros(6))

            self.offset_and_normalize_target(self.target_xyz,
                                             self.fingers_offset)

    def offset_and_normalize_target(self, target, offset=np.zeros(3)):
        # account for offset of fingers from wrist
        R = self.R_func(*(tuple(self.q)))
        self.target_xyz = target + np.dot(R, offset)

        # set it so that target is too far
        magnitude = 0.9
        norm = np.linalg.norm(self.target_xyz)
        if norm > magnitude:
            self.target_xyz = (self.target_xyz / norm * magnitude)

    def start_setup(self):
        raise Exception('start setup method not implemented')

    def start_loop(self):
        raise Exception('start loop method not implemented')

    def print_error(self):
        """ Print out the distance from the end-effector to xyz target """
        hand_xyz = self.robot_config.Tx('EE', q=self.q)
        error = np.sqrt(np.sum((hand_xyz - self.target_xyz)**2))
        print('error: ', error)

    def write_data(self):
        """ Write the data stored in the data dictionary to file """
        for key in self.tracked_data:
            np.savez_compressed('data/%s' % key, self.tracked_data[key])
