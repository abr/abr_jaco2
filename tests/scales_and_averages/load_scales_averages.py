import numpy as np
qs = np.load('q.npz')['q']
dqs = np.load('dq.npz')['dq']
print('q means: ', qs.mean(axis=0))
print('dq means: ', dqs.mean(axis=0))
print('q scales: ', np.amax(qs,axis=0) - np.amin(qs,axis=0))
print('dq scales: ', np.amax(dqs,axis=0) - np.amin(dqs,axis=0))