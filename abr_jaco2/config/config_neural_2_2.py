# to get new scales and means run the get_scales_averages script
# in jaco2/tests
import numpy as np

from . import config


class robot_config(config.robot_config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(**kwargs)

        self.means = {
            'q': np.array([0.37132577, 2.31246158, 2.95651672, 4.55372486,
                0.04542425, 3.14836099]),
            'dq': np.array([-0.02154386, -0.00674915, 0.0113751, -0.02432285,
                -0.00031834, -0.01269092])
            }

        self.scales = {
            'q': np.array([1.4214849, 1.06333844, 1.64560852, 1.80557768,
                0.10418352, 1.62446754]),
            'dq': np.array([0.92301575, 0.81728912, 1.40780448, 2.11406261,
                0.23121436, 1.83917332])
            }

    def scaledown(self, name, x):
        return (x - self.means[name]) / self.scales[name]

    def scaleup(self, name, x):
        return x * self.scales[name] + self.means[name]
