# Plots data from gravity compensation tests
# need to specify which test to plot in filename

import matplotlib.pyplot as plt
import numpy as np
import seaborn

# Load Data
q_desired = np.load('data/gravity_comp/%s/q_desired.npz' % filename)['q_desired']
# array format [num_joints, loop_count, num_positions]
q_actual_3D = np.load('data/gravity_comp/%s/q_actual.npz' % filename)['q_actual']

# set plotting variables based on amount of runs and loop counts in saved data
num_joints = len(q_actual_3D)      # number of joint in arm
num_count = len(q_actual_3D[0])    # number of loop counts arm stays in torque mode 
num_positions = len(q_actual_3D[0][0])   # number of read positions
filename = 'baseline'

plt.figure(figsize=(6, 3))
plt.title('Joint Angles')


q_actual = q_actual_3D[:, :, 1]

# calculate the error
#error_of each joint = q_des - q
# plot this over time, separate plot for each different target position
#total_error_over_time = np.linalg.norm(q_des - q, axis=1)  
#total_error = np.sum(total_error_over_time)

print('q_actual ' , q_actual)
print('q_desired' , q_desired)
"""plt.xlim([0, 500])
plt.xticks(range(0, 500, 50), np.arange(0, 500, 50)*3)
plt.xlabel('~Time (ms)')
plt.ylabel('RSE')
plt.tight_layout()
plt.savefig('data/4.2/results')
plt.show()"""