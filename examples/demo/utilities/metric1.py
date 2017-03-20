""" Generate a metric to compare different runs to """
import matplotlib.pyplot as plt
import numpy as np

folder = '/home/tdewolf/src/parameter_testing/adaptive/baseline/trial0'

# load in data
EE = np.load('%s/EE.npz' % folder)['arr_0']
filtered_target = np.load('%s/filtered_target.npz' % folder)['arr_0']
q = np.load('%s/q.npz' % folder)['arr_0']
dq = np.load('%s/dq.npz' % folder)['arr_0']
target = np.load('%s/target.npz' % folder)['arr_0']
u = np.load('%s/u.npz' % folder)['arr_0']

# calculate total error to target location
total_error = np.sqrt(np.sum((EE - target)**2, axis=0))
print('total error: ', [float('%.4f' % val) for val in total_error])

root_mean_squared_error= np.sqrt(np.mean((EE - target)**2, axis=0))
print('RMS error: ', [float('%.4f' % val) for val in root_mean_squared_error])

for ii in range(3):
    plt.subplot(3, 1, ii+1)
    plt.plot(abs(EE[:, ii] - target[:, ii]))
plt.show()
