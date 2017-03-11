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

# target_xyz = np.array([0.44194174, 0.25659354, 0.91])
target_xyz = np.array([0.46, -0.06, 0.75])

# 3D plot
fig = plt.figure()
ax = fig.add_subplot(221, projection='3d')
target = np.load('data/target.npz')['arr_0']
wrist = np.load('data/wrist.npz')['arr_0']
offset = np.load('data/offset.npz')['arr_0']
print('targets: ', target)
print('wrist: ', wrist)
print('offset: ', offset)
ax.plot(*target.T, 'rx', mew=10, label='target')
ax.plot(*wrist.T, 'g', label='wrist')
ax.plot(*offset.T, 'b', label='fingers')
ax.plot([target_xyz[0]], [target_xyz[1]], [target_xyz[2]], 'gx', mew=10)
ax.plot([0], [0], [0], 'kx', mew=10)  # plot origin
plt.legend()

u = np.linspace(0, 2 * np.pi, 20)
v = np.linspace(0, np.pi, 20)

radius = .7
x = radius * np.outer(np.cos(u), np.sin(v))
y = radius * np.outer(np.sin(u), np.sin(v))
# the .273 is the offset to joint 1
z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + 0.273
ax.plot_surface(x, y, z, rstride=1, cstride=1, color='w', shade=0, alpha=.2)
ax.plot([0], [0], [0.273], 'kx', mew=10)  # plot joint 1 xyz

# plot (x, y)
plt.subplot(2, 2, 2)
plt.plot(target[:, 0], target[:, 1], 'rx', label='target')
plt.plot(wrist[:, 0], wrist[:, 1], 'g', label='wrist')
plt.plot(offset[:, 0], offset[:, 1], 'b', label='fingers')
plt.plot(target_xyz[0], target_xyz[1], 'gx', mew=10)
plt.plot([0], [0], 'kx', mew=10)  # plot origin
plt.plot([0], [0], 'kx', mew=10)  # plot joint 1 xyz
plt.title('(x, y)')

# plot(x, z)
plt.subplot(2, 2, 3)
plt.plot(target[:, 0], target[:, 2], 'rx', label='target')
plt.plot(wrist[:, 0], wrist[:, 2], 'g', label='wrist')
plt.plot(offset[:, 0], offset[:, 2], 'b', label='fingers')
plt.plot(target_xyz[0], target_xyz[2], 'gx', mew=10)
plt.plot([0], [0], 'kx', mew=10)  # plot origin
plt.plot([0], [0.273], 'kx', mew=10)  # plot joint 1 xyz
plt.title('(x, z)')

# plot(y, z)
plt.subplot(2, 2, 4)
plt.plot(target[:, 1], target[:, 2], 'rx', label='target')
plt.plot(wrist[:, 1], wrist[:, 2], 'g', label='wrist')
plt.plot(offset[:, 1], offset[:, 2], 'b', label='fingers')
plt.plot(target_xyz[1], target_xyz[2], 'gx', mew=10)
plt.plot([0], [0], 'kx', mew=10)  # plot origin
plt.plot([0], [0.273], 'kx', mew=10)  # plot joint 1 xyz
plt.title('(y, z)')


plt.show()
