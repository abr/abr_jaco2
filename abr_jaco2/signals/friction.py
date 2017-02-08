import numpy as np

class Signal():
    """ Accounts for the friction in the Jaco2
    """

    def __init__(self, robot_config,
                 fc=None, fv=None, fs=None, c0=None):

        """ Assuming that friction is due to graphite on steel contact
        for motor slip rings, and lubricated steel on steel for any
        bearings in the motor

        Ignoring effects of plastic rings (around motors) on carbon fiber"""

        self.robot_config = robot_config

        self.F_min = np.array([1.75, 0.20, 1.17, 0.92, 0.90, 0.84]) 

        # Normal force
        self.F_n = np.zeros(robot_config.num_joints)    

        # Coulomb friction coeff
        self.fc = (np.zeros(robot_config.num_joints)
                   if fs is None else np.array(fc))
        # viscous friction coeff
        self.fv = (np.zeros(robot_config.num_joints)
                   if fv is None else np.array(fv))
        # fi - fci > 0 is the stiction coeff
        self.fs = (np.zeros(robot_config.num_joints)
                  if fc is None else np.array(fs))
        # positive constant in stiction term
        self.c0 = 0.1*(np.ones(robot_config.num_joints)
                   if c0 is None else np.array(c0))

        # stiction and viscous/kinetic frictional constants
        # slip ring brushes
        self.us_Cu_steel = 0.53
        self.uk_Cu_steel = 0.36
        self.us_graphite_steel = 0.1
        self.uk_graphite_steel = None
        # bearings
        self.us_PE_steel = 0.20
        self.uk_PE_steel = None
        self.us_steel_steel = 0.77  # 0.74-0.80
        self.uk_steel_steel = 0.52  # 0.42-0.62
        self.us_steel_steel_lub = 0.16
        self.uk_steel_steel_lub = 0.07  # 0.03-0.12 (using hard steel)

        self.us = (self.us_graphite_steel + self.us_steel_steel_lub)
        self.uk = self.us_steel_steel_lub 

    def generate(self, dq):
        """ Generates the control signal

        dq np.array: the current joint velocities
        """
        self.calc_F_n()
        self.calc_fc()
        self.calc_fv()
        self.calc_fs()
        friction = (self.fc * np.sign(dq) + self.fv * dq +
                (self.fs - self.fc) * np.exp(-dq / self.c0))
        print('friction: ', friction)
        return friction

    def calc_F_n(self):
        """ calculates the normal force using the minimum force to move joints
        and the provided coefficients of friction"""
        self.F_n = abs(self.F_min / self.us)

    def calc_fc(self):
        """ Calculates the force of friction due to coulombic"""        
        self.fc = self.uk * self.F_n
        

    def calc_fv(self):
        """ Calculates the force of friction due to viscuous effecs"""
        # NOTE: NEED TO FIND COEFFICIENT, SHOULD BE <uk SO APPROXIMATING ATM        
        self.fv = 0.5 * self.uk * self.F_n
        

    def calc_fs(self):
        """ Calculates the force of friction due to static effects"""
        self.fs = self.us * self.F_n