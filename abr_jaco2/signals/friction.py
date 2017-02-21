import numpy as np

class Signal():
    """ Accounts for the friction in the Jaco2
    """

    def __init__(self, robot_config):

        """ Assuming that friction is due to graphite on steel contact
        for motor slip rings, and lubricated steel on steel for any
        bearings in the motor

        Ignoring effects of plastic rings (around motors) on carbon fiber"""

        self.robot_config = robot_config

        # breakaway forces
        self.f_brk = np.array([1.30, 1.20, 0.84, 0.80, 0.75, 0.74])

        # Stribeck friction coefficient
        self.c0 = np.array([0.01, 0.05, 0.05, 0.01, 0.01, 0.01])
        # Coulombic friction force
        self.fc = self.f_brk * np.array([0.005, 0.01, 0.01, 0.01, 0.01, 0.01]) * 0.5
        # viscuous friction force
        self.fv = np.zeros(self.robot_config.num_joints)

        # threshold velocities
        self.v_threshold = np.ones(self.robot_config.num_joints) * 10e-4

    def generate(self, dq):
        """ Generates the control signal

        dq np.array: the current joint velocities
        """

        friction = np.zeros(self.robot_config.num_joints)
        for ii in range(self.robot_config.num_joints):
            if abs(dq[ii]) > self.v_threshold[ii]:
                # normal friction compensation
                friction[ii] = (
                    np.sign(dq[ii]) * (
                        # Coulombic
                        self.fc[ii] +
                        # static
                        (self.f_brk[ii] - self.fc[ii]) *
                        np.exp((-abs(dq[ii]) / self.c0[ii]))) +
                    # viscous
                    self.fv[ii] * dq[ii])
            else:
                # # sub-threshold friction compensation
                friction[ii] = (
                    dq[ii] * (
                        # proportionality coefficient
                        # NOTE: did they typo and write f_v_thresh not f_v?
                        # maybe self.f_brk[ii] / self.v_threshold[ii]
                        # would be f_v_thresh if it wasn't a typo?
                        self.fv[ii] +
                        # Coulombic and static
                        (self.fc[ii] + (self.f_brk[ii] - self.fc[ii]) *
                        np.exp(-self.v_threshold[ii] / self.c0[ii]))) /
                    self.v_threshold[ii])

        return friction
