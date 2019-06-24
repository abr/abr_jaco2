'''
A mock interface for the jaco to test code with the arm out of the loop
'''
import numpy as np

from abr_control.interfaces.interface import Interface as BaseInterface

import os,sys,inspect
current_dir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)


class MockInterface(BaseInterface):
    """ Interface class for the Jaco2 Kinova arm.

    Handles the overhead of interacting with the Cython code,
    conforms the functions to the ABR_Control interface API.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm such as:
        number of joints, number of links, mass information etc.
    """

    def __init__(self, robot_config, display_error_level=2, use_redis=False):
        """ Constructor

        Parameters
        ----------
        display_error_level: int, optional (Default: 3)
            set the level of messages that will be printed to screen
            1. errors, warnings, info, and debugging
            2. errors, warnings, and info
            3: errors and warnings
            4: errors only
            any other int to not display any messages
            NOTE: it is highly recommended to set display_error_level
            to at least 4 to see important error messages
        use_redis: boolean, optional (Default: False)
            True: send joint info to redis server during position mode which is
                  otherwise unavailable during movement
            False: do no import or use redis
        """

        super(MockInterface, self).__init__(robot_config)
        self.display_error_level = display_error_level

    def connect(self):
        """ All initial setup, establish RS485 connection
        """

        print('MOCK: connection made')

    def disconnect(self):
        """ Any socket closing etc that must be done
        """

        print('MOCK: disconnected')

    def get_feedback(self):
        """ Returns a dictionary of relevant information

        Returns the current joint and joint velocity information
        to the controller [radians] [radians/second] respectively
        """

        # convert from degrees from the Jaco into radians
        # Jaco API uses degrees
        if self.display_error_level == 1:
            print('MOCK: position feedback sent')
        feedback = {}
        feedback['q'] = np.ones(6)
        feedback['dq'] = np.ones(6)
        return feedback

    def get_torque_load(self):
        """ Returns the torque at each joint in Nm
        """

        if self.display_error_level == 1:
            print('MOCK: torque feedback sent')
        return np.ones(6)

    def init_force_mode(self):
        """ Changes the arm to torque control mode

        To switch to torque mode the arm must be in a position where the
        torque load at the joints is minimized. Suggested to use the
        Config.INIT_TORQUE_POSITION
        """

        print('MOCK: force mode started')

    def init_position_mode(self):
        """ Changes the arm into position control mode

        Changes the arm into position control mode, where target joint
        angles can be provided, and the motors switch into servo mode.
        """

        print('MOCK: position mode started')

    def open_hand(self, hand_open):
        """ Incrementally open or close the hand

        Parameters
        ----------
        hand_open : boolean
            True to open hand, False to close hand
        """

        assert type(hand_open) == bool
        if self.display_error_level == 1:
            print('MOCK: hand opening/closing')

    def send_forces(self, u):
        """ Applies the set of torques u to the arm.

        NOTE: As part of the Kinova RS485 API, if a torque is not
        applied every 200ms then the arm reverts back to position
        control and the InitForceMode function must be called again.

        Parameters
        ----------
        u : numpy.array
            float value of torques to apply to each joint [Nm]
        """
        assert len(u) == 6
        if self.display_error_level == 1:
            print('MOCK: forces sent')

    def send_target_angles(self, q):
        """ Moves the arm to the specified joint angles

        Moves the arm to the specified joint angles using
        the on-board PD controller.

        q : numpy.array
            the target joint angles [radians]
        """
        assert len(q) == 6
        if self.display_error_level == 1:
            print('MOCK: target angles sent')
