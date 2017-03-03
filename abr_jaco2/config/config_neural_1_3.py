# to get new scales and means run the get_scales_averages script
# in jaco2/tests
import numpy as np

from . import config


class robot_config(config.robot_config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(**kwargs)

        self.means = {
            'q': np.ones(self.num_joints) * np.pi,
            'dq': np.array([-0.01337, 0.00192, 0.00324,
                            0.02502, -0.02226, -0.01342])
            }

        self.scales = {
            'q' : np.ones(self.num_joints) * np.pi,
            'dq': np.array([1.22826, 2.0, 1.42348,
                            2.58221, 2.50768, 1.27004])
            }

    def scaledown(self, name, x):
        return (x - self.means[name]) / self.scales[name]

    def scaleup(self, name, x):
        return x * self.scales[name] + self.means[name]
