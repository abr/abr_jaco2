"""
Implementing the gravity and friction estimation as
described in (Liu and Quach, 2001).
"""
import numpy as np
import sympy as sp
from sympy.utilities.autowrap import autowrap

import abr_control
import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.robot_config(
    regenerate_functions=False, use_cython=True,
    use_simplify=False, hand_attached=False)

# create our VREP interface
interface = abr_jaco2.interface(robot_config)

target_positions = [
    robot_config.home_position + np.ones(robot_config.num_joints) * 0.22,
    robot_config.home_position - np.ones(robot_config.num_joints) * 0.22]
print('target positions: \n', target_positions)

kp = np.diag(np.ones(robot_config.num_joints) * 15)
kv = np.diag(np.ones(robot_config.num_joints) * 3.85)

# described on page 2 of http://bit.ly/2jvNtFR
gear_ratios = np.array([1.0/136.0, 1.0/160.0, 1.0/136.0,
                        1.0/110.0, 1.0/110.0, 1.0/110.0])


def gravity_function(use_cython=False):
    # create the upper triangle W matrix where each element is
    # wij(q) = g.T (dR_0^j/ dq_i) p_j
    # where g = [0, 0, 9.8062].T, R_0^j is the rotation matrix
    # from the origin to joint j, and p_j is the direction in frame i
    # pointing to the centre of mass m_i of link i
    gravity = sp.Matrix([0, 0, 9.8062])  # gravity

    W = []
    for ii in range(robot_config.num_joints-1):
        W.append([])
        for jj in range(robot_config.num_joints-1):
            # calculate the directional vector from joint
            if jj == 0:
                pj  = robot_config.L[0]
            else:
                pj = robot_config.L[(jj*2)+1] + robot_config.L[(jj*2)+2]
            # make it a unit vector
            pj /= np.sum(pj)
            if jj < ii:
                W[ii].append(0)
            else:
                R0j = robot_config._calc_T('link%i' % (jj+1))[:3, :3]
                # fill in the upper triangle
                W[ii].append(
                    gear_ratios[ii] *
                    (gravity.T * R0j.diff(robot_config.q[ii]) * pj)[0, 0])
    W = sp.Matrix(W)
    # sp.pprint(W)
    if use_cython is True:
        return autowrap(W, backend="cython", args=robot_config.q)
    return sp.lambdify(robot_config.q, "numpy")

Wfunc = gravity_function(use_cython=True)

theta_g = np.zeros(robot_config.num_joints-1)
theta_f = np.zeros(robot_config.num_joints-1)

theta_g = (np.array([0.0, -287.993, 88.876, 263.32, -11.189]) +
           np.array([0.0, 191.54442498, -111.30638311, -246.24817009, 6.84653716])+
           np.array([0.0, 166.25957844, 31.73654076, 89.41754829, 22.08351953])+
           np.array([0.0, -1.22510147, 164.05575172, -103.97952572, -16.93228603]))
theta_f = (np.array([0.0, -13.04, -2.776, -0.806, 0.3507])+
           np.array([0.0, 3.50433728, 1.04728242, 0.83638478, -0.19031521])+
           np.array([0.0, 2.18838304e+01, 2.55624759e+00, -5.21767628e-02, 7.66962830e-03])+
           np.array([0.0, -54.29405325, 31.86805291, 0.59431677, 0.15613902]))

try:
    interface.connect()
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    # create a target based on initial arm position
    feedback = interface.get_feedback()
    q = np.array(feedback['q'])
    Wfunc(*q)

    W = np.zeros((2, robot_config.num_joints-1, robot_config.num_joints-1))
    e = np.zeros((2, robot_config.num_joints))
    qs = np.zeros((2, robot_config.num_joints))

    interface.init_force_mode()

    for ii, target_pos in enumerate(target_positions):
        print('Moving to target ', ii+1)
        count = 0
        stable_count = 0
        # move to stable state
        while 1:
            # get arm feedback from VREP
            feedback = interface.get_feedback()
            q = np.array(feedback['q'])
            dq = np.array(feedback['dq'])

            q_tilde = ((target_pos - q + np.pi) % (np.pi * 2)) - np.pi
            u = np.dot(kp, q_tilde) - np.dot(kv, dq)

            g_estimate = np.dot(Wfunc(*q), theta_g)
            u[:-1] += g_estimate + theta_f

            # apply the control signal
            interface.apply_u(np.array(u, dtype='float32'))

            if np.sum(dq**2) < .001:
                # if the arm has reach a stable state, start counting
                stable_count += 1
                if stable_count > 5000:
                    # assume now it's actually a stable state
                    break
                if stable_count % 1000 == 0:
                    print('stable count: ', stable_count)
            else:
                stable_count = 0
            if count % 100 == 0:
                print('error: ', target_pos - q)
            count += 1
        # store stable position
        qs[ii] = np.copy(q)
        # from stable state, calculate gravity basis function
        W[ii] = Wfunc(*q)
        # calculate error between desired state and stable state
        e[ii] = target_pos - q
        print('Data collected')

    print('qs: ', qs)
    print('errors: ', e)

    # calculate gravity parameters, cut out first column and row
    dW = (W[0] - W[1])[1:, 1:]
    e = e[:, 1:-1]
    # make sure that W is non-singular
    assert np.linalg.matrix_rank(dW) == dW.shape[0]

    W_inv = np.dot(dW.T, np.linalg.pinv(np.dot(dW, dW.T)))
    theta_g = np.dot(W_inv, np.dot(kp[1:-1, 1:-1], e[0] - e[1]))
    # calculate friction parameters
    theta_f = np.dot(kp[1:-1, 1:-1], e[0]) - np.dot(W[0][1:, 1:], theta_g)

finally:
    # stop and reset the VREP simulation
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    interface.disconnect()

    print('theta_g: ', theta_g)
    print('theta_f: ', theta_f)
