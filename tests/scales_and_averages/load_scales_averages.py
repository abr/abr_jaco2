import numpy as np

qs = np.load('q.npz')['q']
dqs = np.load('dq.npz')['dq']
print('q means: ', [float('%.5f' % val) for val in qs.mean(axis=0)])
print('dq means: ', [float('%.5f' % val) for val in dqs.mean(axis=0)])
print('q scales: ', [float('%.5f' % val) for val in
                     (np.amax(qs,axis=0) - np.amin(qs,axis=0))])
print('dq scales: ', [float('%.5f' % val) for val in
                      (np.amax(dqs,axis=0) - np.amin(dqs,axis=0))])

import matplotlib.pyplot as plt
plt.subplot(2, 1, 1)
plt.plot(qs)
plt.title('joint angles')
plt.legend(range(6))
plt.subplot(2, 1, 2)
plt.plot(dqs)
plt.title('joint velocities')
plt.legend(range(6))
plt.show()
