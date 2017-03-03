import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import abr_jaco2

target = np.load('data/target.npz')['arr_0']
wrist = np.load('data/wrist.npz')['arr_0']
#fingers = np.load('data/fingers.npz')['arr_0']

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(*target.T, 'rx', mew=10)
ax.plot(*wrist.T, 'g')
#ax.plot(*fingers.T, 'b')

plt.show()
