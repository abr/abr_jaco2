import numpy as np
import sympy as sp
import time

import abr_control
import abr_jaco2

def gen_rotation_matrix(angles):
    print(np.cos(1.0))
    alpha = angles[0]
    print('alpha: ', alpha)
    beta = angles[1]
    print('beta: ', beta)
    gamma = angles[2]
    print('gamma: ', gamma)

    print(np.cos(alpha))

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]])
    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]])
    Rz = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]])

    ans = np.dot(Rx, np.dot(Ry, Rz))
    ans[abs(ans) < 1e-5] = 0
    print('Regenerated: \n', ans)

#gen_rotation_matrix([0, np.pi/2.0, np.pi/2.0])


# robot_config = abr_control.arms.onelink.config(
robot_config = abr_jaco2.robot_config(
    regenerate_functions=True, use_cython=True,
    use_simplify=False, hand_attached=False)

interface = abr_control.interfaces.vrep(
    robot_config=robot_config, dt=.001)
interface.connect()

# ctrlr = abr_control.controllers.osc(
#     robot_config=robot_config)

name = 'link1'
try:
    # test out our orientation calculations
    while 1:
        feedback = interface.get_feedback()
        print('q: ', feedback['q'])
        xyz = robot_config.Tx(name, q=feedback['q'])
        print('xyz: ', xyz)
        #angles = robot_config.orientation(name, q=feedback['q'])

        #print('angles: ', np.array(angles).T * 180 / np.pi)
        # angles = np.array([0, -10, 90]) * np.pi / 180

        interface.set_xyz('hand', xyz)
        #interface.set_orientation('hand', angles)
        # print(T_func(*tuple(feedback['q'])))
        #gen_rotation_matrix(angles)
        # print(interface.get_orientation('hand'))
        # interface.set_orientation('hand', interface.get_orientation('Disc')[1])
        time.sleep(1)
        
        #T = robot_config._calc_T(name=name)
        #T_func = sp.lambdify(robot_config.q, T, "numpy")
        #T = T_func(*feedback['q'])
        #print('T: \n', T)

finally:
    # stop and reset the VREP simulation
    interface.disconnect()
