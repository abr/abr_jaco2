import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import abr_jaco2

#qs = np.load('data/q.npz')['arr_0']
#dqs = np.load('data/dq.npz')['arr_0']

#plt.figure()
#plt.subplot(2, 1, 1)
#plt.plot(qs)
#plt.subplot(2, 1, 2)
#plt.plot(dqs)

target = np.load('data/target.npz')['arr_0']
wrist = np.load('data/wrist.npz')['arr_0']
offset = np.load('data/offset.npz')['arr_0']
print('targets: ', target)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(*target.T, 'rx', mew=10)
ax.plot(*wrist.T, 'g')
ax.plot(*offset.T, 'b')

plt.show()
