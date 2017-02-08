import numpy as np

class Signal():
    """ Accounts for the stiction we've found in the Jaco2
    """

    def __init__(self, robot_config):

        self.robot_config = robot_config

        # the minimum force inputs to each motor to induce movement
        self.u_min = [1.75, 0.20, 1.17, 0.92, 0.90, 0.84]

    def generate(self, dq, u):
        """ Generates the control signal

        dq np.array: the current joint velocities
        """

        """u_add = np.zeros(self.robot_config.num_joints)
        for ii in range(self.robot_config.num_joints):
            if abs(u[ii]) > 1e-4 and abs(dq[ii]) < 1e-2 and abs(u[ii]) < self.u_min[ii]:
                u_add[ii] = self.u_min[ii] * np.sign(u[ii])
        return u_add"""
        print('Nothing computer for stiction')
        return np.zeros(self.robot_config.num_joints)
