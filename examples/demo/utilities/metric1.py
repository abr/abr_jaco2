""" Generate a metric to compare different runs to """
import matplotlib.pyplot as plt
import numpy as np
import glob

data_location = 'parameter_testing'
folder = '../data/' + data_location + '/**/trial0/'

plt.figure()
total_folders = 0
error_track=[]
rms_error_track=[]
name_track=[]

for filename in glob.iglob(folder, recursive=True):
    # load in data
    EE = np.load('%s/EE.npz' % filename)['arr_0']
    filtered_target = np.load('%s/filtered_target.npz' % filename)['arr_0']
    q = np.load('%s/q.npz' % filename)['arr_0']
    dq = np.load('%s/dq.npz' % filename)['arr_0']
    target = np.load('%s/target.npz' % filename)['arr_0']
    filtered_target = np.load('%s/filtered_target.npz' % filename)['arr_0']
    u = np.load('%s/u.npz' % filename)['arr_0']
    u_adapt = np.load('%s/u_adapt.npz' % filename)['arr_0']
    training_signal = np.load('%s/training_signal.npz' % filename)['arr_0']

    print('\n' + filename[25:-8])
    # calculate total error to target location
    total_error = np.sqrt(np.sum((EE - target)**2, axis=0))
    print('total error: ', [float('%.4f' % val) for val in total_error])

    total_error_sum = np.sum(np.sqrt(np.sum((EE - target)**2, axis=1)))
    print('total error sum: ', total_error_sum)

    rms_error= np.sqrt(np.mean((EE - target)**2, axis=0))
    print('RMS error: ', [float('%.4f' % val) for val in rms_error])

    rms_error_sum= np.sum(np.sqrt(np.mean((EE - target)**2, axis=0)))
    print('RMS error sum: ', rms_error_sum)

    error_track.append(np.copy(total_error_sum))
    rms_error_track.append(np.copy(rms_error_sum))
    name_track.append(np.copy(filename[25:-8]))

y_pos = np.arange(len(error_track))

plt.subplot(2, 1, 1)
plt.title('Error')
plt.barh(y_pos, error_track, align='center')
plt.yticks(y_pos, name_track)
plt.xlabel('Total Error')

plt.subplot(2, 1, 2)
plt.title('RMS Error')
plt.barh(y_pos, rms_error_track, align='center')
plt.yticks(y_pos, name_track)
plt.xlabel('Total RMS Error')

plt.show()
    # for ii in range(3):
    #     plt.subplot(3, 1, ii+1)
    #     plt.plot(abs(EE[:, ii] - target[:, ii]))
    # plt.show()
