""" Generate a metric to compare different runs to """
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import abr_jaco2

rc = abr_jaco2.robot_config()

folder = '/home/tdewolf/src/parameter_testing/adaptive/baseline/trial0'

# load in data
EE = np.load('%s/EE.npz' % folder)['arr_0']
q = np.load('%s/q.npz' % folder)['arr_0']
dq = np.load('%s/dq.npz' % folder)['arr_0']
target = np.load('%s/target.npz' % folder)['arr_0']
u = np.load('%s/u.npz' % folder)['arr_0'] * .004  # .004 is the approximate time step
u_adapt = np.load('%s/u_adapt.npz' % folder)['arr_0'] * .004  # .004 is the approximate time step

fig = plt.figure(figsize=(6, 6))
fig.add_subplot(1, 1, 1, projection='3d')

plt.plot(EE[:, 0], EE[:, 1], EE[:, 2], 'r--')
plt.plot(target[:, 0], target[:, 1], target[:, 2], 'rx', mew=10)
for ii in range(0, q.shape[0], 20):
    # calculate the Jacobian
    J = rc.J('EE', q=q[ii])
    # transform from 6D space to 3D
    u3D = np.dot(np.linalg.pinv(J.T), u_adapt[ii]) * 10
    # get position of end-effector
    xyz = rc.Tx('EE', q=q[ii], x=[0, 0, 0.12])

    # plot force from ee xyz outward
    plt.quiver(xyz[0], xyz[1], xyz[2],
               u3D[0], u3D[1], u3D[2])
plt.plot([0], [0], [0])
plt.plot([0], [2], [2])
plt.plot([0], [-2], [2])
plt.show()
