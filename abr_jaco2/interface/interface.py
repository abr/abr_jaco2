import numpy as np

from abr_control.interfaces.interface import Interface as BaseInterface
from . import jaco2_rs485


class Interface(BaseInterface):
    """ Interface class for the Jaco2 Kinova arm.

    Handles the overhead of interacting with the cython code,
    conforms the functions to the abr_control interface system.

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    """

    def __init__(self, robot_config):
        super(Interface, self).__init__(robot_config)
        self.jaco2 = jaco2_rs485.pyJaco2()

    def connect(self):
        """ All initial setup, establish RS485 connection
        """

        self.jaco2.Connect()

    def disconnect(self):
        """ Any socket closing etc that must be done
        """

        self.jaco2.Disconnect()

    def get_feedback(self):
        """ Returns a dictionary of relevant information

        Returns the current joint and joint velocity information
        to the controller [radians] [radians/second] respectively
        """

        # convert from degrees from the Jaco into radians
        # Jaco API uses degrees
        feedback = self.jaco2.GetFeedback()
        feedback['q'] = (np.array(feedback['q']) * np.pi / 180.0) % (2 * np.pi)
        feedback['dq'] = np.array(feedback['dq']) * np.pi / 180.0
        return feedback

    def get_torque_load(self):
        """ Returns the torque at each joint in Nm
        """

        return self.jaco2.GetTorqueLoad()

    def init_force_mode(self):
        """ Changes the arm to torque control mode

        To switch to torque mode the arm must be in a position where the
        torque load at the joints is minimized. Pointing straight up or at
        minimal angles works best. Using Config.INIT_TORQUE_POSITION works
        the best
        """

        self.jaco2.InitForceMode()

    def init_position_mode(self):
        """ Changes the arm into position control mode

        Changes the arm into position control mode, where
        target joint angles can be provided, and the on-board PD
        controller will move the arm.
        """

        self.jaco2.InitPositionMode()

    def open_hand(self, hand_open):
        """ Opens and closes the hand incrementally

        moves in increments for each function call

        Parameters
        ----------
        hand_open : boolean
            True to open hand, False to close hand
        """

        self.jaco2.SendTargetAnglesHand(hand_open)

    def send_forces(self, u):
        """ Applies the set of torques u to the arm.

        NOTE: if a torque is not applied every 200ms then
        the arm reverts back to position control and the
        InitForceMode function must be called again.

        Parameters
        ----------
        u : numpy.array
            float value of torques to apply to each joint [Nm]
        """
        self.jaco2.SendForces(u)

    def send_target_angles(self, q):
        """ Moves the arm to the specified joint angles

        Moves the arm to the specified joint angles using
        the on-board PD controller.

        q : numpy.array
            the target joint angles [radians]
        """
        # TODO: need to account for negative degrees
        # convert from radians into degrees the Jaco expects
        q = np.array(q) * 180.0 / np.pi
        self.jaco2.SendTargetAngles(q)
