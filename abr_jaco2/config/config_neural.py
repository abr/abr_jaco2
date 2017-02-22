# to get new scales and means run the get_scales_averages script
# in jaco2/tests
import numpy as np

from . import config


class robot_config(config.robot_config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(**kwargs)

        self.means = {
            'q': np.array([ 0.700, 3.671, 3.060, 4.499, -0.410, 3.076]),
            'dq': np.array([-0.006, 0.014, 0.016, -0.048, -0.001, -0.005])
            }

        self.scales = {
            'q': np.array([ 1.017, 1.820, 2.048, 7.847, 6.176, 1.642]),
            'dq': np.array([ 0.829, 0.644, 1.436, 2.143, 2.334, 1.464])
            }

    def scaledown(self, name, x):
        return (x - self.means[name]) / self.scales[name]

    def scaleup(self, name, x):
        return x * self.scales[name] + self.means[name]