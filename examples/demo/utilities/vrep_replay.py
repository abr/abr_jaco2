"""
send recorded data to vrep display to replay a test script
Once vrep_display.py is running, running this script will
send joint angle and target information to replay the run
with it's data saved in 'file_location'
"""
import numpy as np
import redis
import time

redis_server = redis.StrictRedis(host='localhost')
file_location = 'parameter_testing/adaptive/baseline/trial0'
loop_delay = 0.0045

q = np.load('../data/%s/q.npz' % file_location)['arr_0']
filtered_target = np.load('../data/%s/filtered_target.npz' % file_location)['arr_0']

for ii in range(0, len(q)):
    redis_server.set('q', '%.3f %.3f %.3f %.3f %.3f %.3f' %
                     (q[ii,0],q[ii,1],q[ii,2],
                      q[ii,3],q[ii,4],q[ii,5]))

    redis_server.set(
        'norm_target_xyz_robot_coords', '%.3f %.3f %.3f'
        % (filtered_target[ii,0],
        filtered_target[ii,1],
        filtered_target[ii,2]))
    time.sleep(loop_delay)
