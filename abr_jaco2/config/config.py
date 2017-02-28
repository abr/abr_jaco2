import numpy as np
import os
import sympy as sp

from abr_control.arms import robot_config
import abr_jaco2


class robot_config(robot_config.robot_config):
    """ Robot config file for the Kinova Jaco^2 V2"""

    def __init__(self, hand_attached=True, **kwargs):

        self.hand_attached = hand_attached
        num_links = 7 if hand_attached is True else 6
        super(robot_config, self).__init__(num_joints=6, num_links=num_links,
                                           robot_name='jaco2', **kwargs)

        self.demo_tooltip_read_pos = np.array(
            [1.80, 3.26, 2.60, 1.04, 2.26, 1.65],
            dtype='float32')

        self.demo_pos_xyz = np.array([.40, -.18, .85])

        self.demo_pos_q = np.array(
            [0.36, 2.19, 2.63, 4.69, 0.024, 3.16],
            dtype="float32")

        self._T = {}  # dictionary for storing calculated transforms

        self.F_brk = np.array([1.40, 0.85, 0.84, 0.80, 0.75, 0.74])

        self.config_folder = (os.path.dirname(abr_jaco2.config.__file__) +
                              '/saved_functions_')
        if self.hand_attached is True:
            self.config_folder += 'with_hand'
        else:
            self.config_folder += 'no_hand'

        self.joint_names = ['joint%i' % ii
                            for ii in range(self.num_joints)]
        # Kinova Home Position - straight up
        self.home_position_start = np.array(
            [1.22, 2.79, 2.62, 4.71, 0.0, 3.14],
            dtype="float32")
        self.home_position_end = np.array(
            [1.22, 3.14, 3.14, 4.71, 0.0, 3.14],
            dtype="float32")

        # for the null space controller, keep arm near these angles
        # currently set to the center of the limits
        self.rest_angles = np.array(
            [None, 2.42, 2.42, 0.0, 0.0, 0.0], dtype='float32')
        self.mass_multiplier_wrist = 1.3

        # create the inertia matrices for each link of the kinova jaco2
        self._M_links = [
            sp.Matrix([  # link0
                [0.640, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.640, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.640, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 0.010, 0.010, 0.010],
                [0.000, 0.000, 0.000, 0.010, 0.010, 0.010],
                [0.000, 0.000, 0.000, 0.010, 0.010, 0.010]]),
            sp.Matrix([  # link1
                [0.182, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.182, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.182, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 9.99e-5, 0.0, -2.95e-5],
                [0.000, 0.000, 0.000, 0.0, 3.216e-4, 0.0],
                [0.000, 0.000, 0.000, 8.32e-5, 0.0, 3.519e-4]]),
            sp.Matrix([  # link2
                [0.424, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.424, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.424, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 0.000, 0.000, 3.786e-3],
                [0.000, 0.000, 0.000, 0.000, -3.725e-3, 0.000],
                [0.000, 0.000, 0.000, 6.97e-5, 0.000, 0.000]]),
            sp.Matrix([  # link3
                [0.211, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.211, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.2113, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, -1.44e-7, 0.000, 5.11e-4],
                [0.000, 0.000, 0.000, 0.000, -4.8e-4, 0.000],
                [0.000, 0.000, 0.000, 4.81e-5, 0.000, 1.533e-6]]),
            sp.Matrix([  # link4
                [0.069, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.069, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.069, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 1.72e-5, 1.815e-5, 0.000],
                [0.000, 0.000, 0.000, 0.000, 0.000, -3.89e-5],
                [0.000, 0.000, 0.000, -9.87e-6 ,3.16e-5 ,0.000]]),
            sp.Matrix([  # link5
                [0.069, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.069, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.069, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 1.72e-5, 1.815e-5, 0.000],
                [0.000, 0.000, 0.000, 0.000, 0.000, -3.89e-5],
                [0.000, 0.000, 0.000, -9.87e-6 ,3.16e-5 ,0.000]])]
        if self.hand_attached is True:
            self._M_links.append(sp.Matrix([  # hand
                [0.727, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.727, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.727, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 4.87e-6, 4.65e-5, 0.000],
                [0.000, 0.000, 0.000, -2.51e-5, 9e-6 , 1.04e-6],
                [0.000, 0.000, 0.000, 5.13e-7, 0.000, 5.25e-5]]))

        self._M_joints = [  # mass of rings added
            sp.Matrix([  # motor 0
                [0.586, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.586, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.586, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 0.000, 2.44e-4, -2.49e-5],
                [0.000, 0.000, 0.000, 2.44e-6, 2.2e-5, 2.77e-4],
                [0.000, 0.000, 0.000, 2.44e-4, 0.000, -2.76e-6]]),
            sp.Matrix([  # motor 1
                [0.572, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.572, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.572, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 0.000, 1.98e-4, -1.5e-4],
                [0.000, 0.000, 0.000, 2.39e-6, 1.32e-4, 2.27e-4],
                [0.000, 0.000, 0.000, 2.39e-4, 0.000, -2.37e-6]]),
            sp.Matrix([  # motor2
                [0.586, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.586, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.586, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 0.000, 2.44e-4, -2.49e-5],
                [0.000, 0.000, 0.000, 2.44e-6, 2.2e-5, 2.77e-4],
                [0.000, 0.000, 0.000, 2.44e-4, 0.000, -2.76e-6]]),
            sp.Matrix([  # motor3
                [0.348*self.mass_multiplier_wrist, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.348*self.mass_multiplier_wrist, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.348*self.mass_multiplier_wrist, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 3.58e-6, 5.03e-5, 1.09e-4],
                [0.000, 0.000, 0.000, 3.22e-5, -1.05e-4, 4.79e-5],
                [0.000, 0.000, 0.000, 1.14e-4, 2.75e-5, -1.68e-5]]),
            sp.Matrix([  # motor4
                [0.348*self.mass_multiplier_wrist, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.348*self.mass_multiplier_wrist, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.348*self.mass_multiplier_wrist, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 3.58e-6, 5.03e-5, 1.09e-4],
                [0.000, 0.000, 0.000, 3.22e-5, -1.05e-4, 4.79e-5],
                [0.000, 0.000, 0.000, 1.14e-4, 2.75e-5, -1.68e-5]]),
            sp.Matrix([  # motor5
                [0.348*self.mass_multiplier_wrist, 0.000, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.348*self.mass_multiplier_wrist, 0.000, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.348*self.mass_multiplier_wrist, 0.000, 0.000, 0.000],
                [0.000, 0.000, 0.000, 3.58e-6, 5.03e-5, 1.09e-4],
                [0.000, 0.000, 0.000, 3.22e-5, -1.05e-4, 4.79e-5],
                [0.000, 0.000, 0.000, 1.14e-4, 2.75e-5, -1.68e-5]])]

        # segment lengths associated with each transform
        # ignoring lengths < 1e-6
        self.L = [
            [0.0, 0.0, 7.8369e-02],  # link 0 offset
            [-3.2712e-05, -1.7324e-05, 7.8381e-02],  # joint 0 offset
            [2.1217e-05, 4.8455e-05, -7.9515e-02],  # link 1 offset
            [-2.2042e-05, 1.3245e-04, -3.8863e-02],  # joint 1 offset
            [-1.9519e-03, 2.0902e-01, -2.8839e-02],  # link 2 offset
            [-2.3094e-02, -1.0980e-06, 2.0503e-01],  # joint 2 offset
            [-4.8786e-04, -8.1945e-02, -1.2931e-02],  # link 3 offset
            [2.5923e-04, -3.8935e-03, -1.2393e-01],  # joint 3 offset
            [-4.0053e-04, 1.2581e-02, -3.5270e-02],  # link 4 offset
            [-2.3603e-03, -4.8662e-03, 3.7097e-02],  # joint 4 offset
            [-5.2974e-04, 1.2272e-02, -3.5485e-02],  # link 5 offset
            [-1.9534e-03, 5.0298e-03, -3.7176e-02]]  # joint 5 offset
        if self.hand_attached is True:  # add in hand offset
            self.L.append([0.000684, 0.0, 0.001])  # com of the hand
            # TODO: need to add in the offset for the fingers
        self.L = np.array(self.L)

        self.L_motors = [
            [0.0, 0.0, -0.00856],   # motor0
            [0.0, 0.0, -0.00806],   # motor1
            [0.0, 0.0, -0.00856],   # motor2
            [0.0, 0.0, -0.00566],   # motor3
            [0.0, 0.0, -0.00566],   # motor4
            [0.0, 0.0, -0.00566]]   # motor5

        self.L_motors = np.array(self.L_motors)
        # ---- Transform Matrices ----

        # Transform matrix : origin -> link 0
        # no change of axes, account for offsets
        self.Torgl0 = sp.Matrix([
            [1, 0, 0, self.L[0, 0]],
            [0, 1, 0, self.L[0, 1]],
            [0, 0, 1, self.L[0, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : link0 -> joint 0
        # account for change of axes and offsets
        self.Tl0j0 = sp.Matrix([
            [1, 0, 0, self.L[1, 0]],
            [0, -1, 0, self.L[1, 1]],
            [0, 0, -1, self.L[1, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> link 1
        # account for rotations due to q
        self.Tj0l1a = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for change of axes and offsets
        self.Tj0l1b = sp.Matrix([
            [-1, 0, 0, self.L[2, 0]],
            [0, -1, 0, self.L[2, 1]],
            [0, 0, 1, self.L[2, 2]],
            [0, 0, 0, 1]])
        self.Tj0l1 = self.Tj0l1a * self.Tj0l1b

        # Transform matrix : link 1 -> joint 1
        # account for axes rotation and offset
        self.Tl1j1 = sp.Matrix([
            [1, 0, 0, self.L[3, 0]],
            [0, 0, -1, self.L[3, 1]],
            [0, 1, 0, self.L[3, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 1 -> link 2
        # account for rotations due to q
        self.Tj1l2a = sp.Matrix([
            [sp.cos(self.q[1]), -sp.sin(self.q[1]), 0, 0],
            [sp.sin(self.q[1]), sp.cos(self.q[1]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for axes rotation and offsets
        self.Tj1l2b = sp.Matrix([
            [0, -1, 0, self.L[4, 0]],
            [0, 0, 1, self.L[4, 1]],
            [-1, 0, 0, self.L[4, 2]],
            [0, 0, 0, 1]])
        self.Tj1l2 = self.Tj1l2a * self.Tj1l2b

        # Transform matrix : link 2 -> joint 2
        # account for axes rotation and offsets
        self.Tl2j2 = sp.Matrix([
            [0, 0, 1, self.L[5, 0]],
            [1, 0, 0, self.L[5, 1]],
            [0, 1, 0, self.L[5, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 2 -> link 3
        # account for rotations due to q
        self.Tj2l3a = sp.Matrix([
            [sp.cos(self.q[2]), -sp.sin(self.q[2]), 0, 0],
            [sp.sin(self.q[2]), sp.cos(self.q[2]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for axes rotation and offsets
        self.Tj2l3b = sp.Matrix([
            [0.14262926, -0.98977618, 0, self.L[6, 0]],
            [0, 0, 1, self.L[6, 1]],
            [-0.98977618, -0.14262926, 0, self.L[6, 2]],
            [0, 0, 0, 1]])
        self.Tj2l3 = self.Tj2l3a * self.Tj2l3b

        # Transform matrix : link 3 -> joint 3
        # account for axes change and offsets
        self.Tl3j3 = sp.Matrix([
            [-0.14262861, -0.98977628, 0, self.L[7, 0]],
            [0.98977628, -0.14262861, 0, self.L[7, 1]],
            [0, 0, 1, self.L[7, 2]],
            [0, 0, 0, 1]])

        # Transform matrix: joint 3 -> link 4
        # account for rotations due to q
        self.Tj3l4a = sp.Matrix([
            [sp.cos(self.q[3]), -sp.sin(self.q[3]), 0, 0],
            [sp.sin(self.q[3]), sp.cos(self.q[3]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for axes and rotation and offsets
        self.Tj3l4b = sp.Matrix([
            [0.85536427, -0.51802699, 0, self.L[8, 0]],
            [-0.45991232, -0.75940555,  0.46019982, self.L[8, 1]],
            [-0.23839593, -0.39363848, -0.88781537, self.L[8, 2]],
            [0, 0, 0, 1]])
        self.Tj3l4 = self.Tj3l4a * self.Tj3l4b

        # Transform matrix: link 4 -> joint 4
        # no axes change, account for offsets
        self.Tl4j4 = sp.Matrix([
            [-0.855753802, 0.458851168, 0.239041914, self.L[9, 0]],
            [0.517383113, 0.758601438, 0.396028500, self.L[9, 1]],
            [0, 0.462579144, -0.886577910, self.L[9, 2]],
            [0, 0, 0, 1]])

        # Transform matrix: joint 4 -> link 5
        # account for rotations due to q
        self.Tj4l5a = sp.Matrix([
            [sp.cos(self.q[4]), -sp.sin(self.q[4]), 0, 0],
            [sp.sin(self.q[4]), sp.cos(self.q[4]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for axes and rotation and offsets
        # no axes change, account for offsets
        self.Tj4l5b = sp.Matrix([
            [0.89059413, 0.45479896, 0, self.L[10, 0]],
            [-0.40329059, 0.78972966, -0.46225942, self.L[10, 1]],
            [-0.2102351, 0.41168552, 0.88674474, self.L[10, 2]],
            [0, 0, 0, 1]])
        self.Tj4l5 = self.Tj4l5a * self.Tj4l5b

        # Transform matrix : link 5 -> joint 5
        # account for axes change and offsets
        self.Tl5j5 = sp.Matrix([
            [-0.890598824, 0.403618758, 0.209584432, self.L[11, 0]],
            [-0.454789710, -0.790154512, -0.410879747, self.L[11, 1]],
            [0, -0.461245863, 0.887272337, self.L[11, 2]],
            [0, 0, 0, 1]])

        # Transform matrix: joint 5 -> link 6
        # account for rotations due to q
        if self.hand_attached is True:
            self.Tj5l6a = sp.Matrix([
                [sp.cos(self.q[5]), -sp.sin(self.q[5]), 0, 0],
                [sp.sin(self.q[5]), sp.cos(self.q[5]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])
            # no axes change, account for offsets
            # NOTE: why are the x and y axes flipped?
            self.Tj5l6b = sp.Matrix([
                [-1, 0, 0, self.L[12, 0]],
                [0, -1, 0, self.L[12, 1]],
                [0, 0, 1, self.L[12, 2]],
                [0, 0, 0, 1]])
            self.Tj5l6 = self.Tj5l6a * self.Tj5l6b

        self.Torgcama = sp.Matrix([
            [sp.cos(-np.pi/4.0), -np.sin(-np.pi/4.0), 0.0, 0.0],
            [sp.sin(-np.pi/4.0), sp.cos(-np.pi/4.0), 0, -0.29],
            [0, 0, 1, 1.01],
            [0, 0, 0, 1]])

        self.Torgcamb = sp.Matrix([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]])
        self.Torgcam = self.Torgcama * self.Torgcamb

        # ---- Motor Transform Matrices ----

        # Transform matrix : joint0 -> motor0
        # no change of axes, account for offsets
        self.Tj0m0 = sp.Matrix([
            [1, 0, 0, self.L_motors[0, 0]],
            [0, 1, 0, self.L_motors[0, 1]],
            [0, 0, 1, self.L_motors[0, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint1 -> motor1
        # no change of axes, account for offsets
        self.Tj1m1 = sp.Matrix([
            [1, 0, 0, self.L_motors[1, 0]],
            [0, 1, 0, self.L_motors[1, 1]],
            [0, 0, 1, self.L_motors[1, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint2 -> motor2
        # no change of axes, account for offsets
        self.Tj2m2 = sp.Matrix([
            [1, 0, 0, self.L_motors[2, 0]],
            [0, 1, 0, self.L_motors[2, 1]],
            [0, 0, 1, self.L_motors[2, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint3 -> motor3
        # no change of axes, account for offsets
        self.Tj3m3 = sp.Matrix([
            [1, 0, 0, self.L_motors[3, 0]],
            [0, 1, 0, self.L_motors[3, 1]],
            [0, 0, 1, self.L_motors[3, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint4 -> motor4
        # no change of axes, account for offsets
        self.Tj4m4 = sp.Matrix([
            [1, 0, 0, self.L_motors[4, 0]],
            [0, 1, 0, self.L_motors[4, 1]],
            [0, 0, 1, self.L_motors[4, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint5 -> motor5
        # no change of axes, account for offsets
        self.Tj5m5 = sp.Matrix([
            [1, 0, 0, self.L_motors[5, 0]],
            [0, 1, 0, self.L_motors[5, 1]],
            [0, 0, 1, self.L_motors[5, 2]],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for orientations)
        kz = sp.Matrix([0, 0, 1])
        self.J_orientation = [
            self._calc_T('joint0')[:3, :3] * kz,  # joint 0 angular velocity
            self._calc_T('joint1')[:3, :3] * kz,  # joint 1 angular velocity
            self._calc_T('joint2')[:3, :3] * kz,  # joint 2 angular velocity
            self._calc_T('joint3')[:3, :3] * kz,  # joint 3 angular velocity
            self._calc_T('joint4')[:3, :3] * kz,  # joint 4 angular velocity
            self._calc_T('joint5')[:3, :3] * kz]  # joint 5 angular velocity

    def _calc_T(self, name):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """

        # TODO: use recursion to reduce this function size
        if self._T.get(name, None) is None:
            if name == 'link0':
                self._T[name] = self.Torgl0
            elif name == 'joint0':
                self._T[name] = self.Torgl0 * self.Tl0j0 * self.Tj0m0
            elif name == 'link1':
                self._T[name] = self.Torgl0 * self.Tl0j0 * self.Tj0l1
            elif name == 'joint1':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1m1)
            elif name == 'link2':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2)
            elif name == 'joint2':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2m2)
            elif name == 'link3':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3)
            elif name == 'joint3':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                    self.Tj3m3)
            elif name == 'link4':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                    self.Tj3l4)
            elif name == 'joint4':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                    self.Tj3l4 * self.Tl4j4 * self.Tj4m4)
            elif name == 'link5':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                    self.Tj3l4 * self.Tl4j4 * self.Tj4l5)
            elif name == 'joint5':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                    self.Tj3l4 * self.Tl4j4 * self.Tj4l5 * self.Tl5j5 *
                    self.Tj5m5)
            elif name == 'EE' and self.hand_attached is False:
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                    self.Tj3l4 * self.Tl4j4 * self.Tj4l5 * self.Tl5j5)
            elif (self.hand_attached is True and
              (name == 'EE' or name == 'link6')):
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                    self.Tj3l4 * self.Tl4j4 * self.Tj4l5 * self.Tl5j5 *
                    self.Tj5l6)
            elif name == 'camera':
                self._T[name] = self.Torgcam

            else:
                raise Exception('Invalid transformation name: %s' % name)

        return self._T[name]
