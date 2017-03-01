"""
A class to use for the demo scripts. Handles the common code.
"""
import numpy as np

import abr_jaco2


class Demo(object):

    def __init__(self):
        self.track = {'q': [],
                      'dq': []}

        # create our interface for the jaco2
        self.interface = abr_jaco2.interface(self.robot_config)
        # connect to the jaco
        self.interface.connect()
        # move to the home position
        self.interface.apply_q(self.robot_config.home_position_start)

        self.count = 0
        self.move_home = False
        self.start_movement = False
        print('Arm Ready')

    def run(self):
        # set up key input tracker
        self.kb = abr_jaco2.KBHit()

        self.count = 0
        while 1:

            if self.start_movement is True:
                # get feedback
                feedback = self.interface.get_feedback()
                self.q = np.array(feedback['q'])
                self.dq = np.array(feedback['dq'])

                self.start_loop()

            if self.move_home is True:
                self.interface.apply_q(self.robot_config.home_position_start)
                self.move_home = False

            if self.kb.kbhit():
                c = self.kb.getch()
                if ord(c) == 112:  # letter p, closes hand
                    self.interface.open_hand(False)
                if ord(c) == 111:  # letter o, opens hand
                    self.interface.open_hand(True)
                if ord(c) == 115:  # letter s, starts movement
                    self.start_movement = True
                    # switch to torque control mode
                    self.interface.init_force_mode()
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
        self.interface.apply_q(self.robot_config.home_position_start)
        # close the connection to the arm
        self.interface.disconnect()
        self.kb.set_normal_term()

    def generate_u(self):
        raise Exception('generate_u method not implemented')

    def print_error(self):
        hand_xyz = self.robot_config.Tx('EE', q=self.q)
        error = np.sqrt(np.sum((hand_xyz - self.target_xyz)**2))
        print('error: ', error)
