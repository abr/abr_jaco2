"""
Checks redis for 'get_target' confirmation, then sends
arm to three positions, waiting 10 seconds between points
"""
import numpy as np
import redis
import time

# create a server for the vision system to connect to
redis_server = redis.StrictRedis(host='localhost')
redis_server.set("controller_name", "Adaptive")

target_positions = [[0.195,-0.070,0.510],
                    [0.028,0.094,0.753],
                    [-0.198,-0.076,0.592]]

first_pass = True
get_target = redis_server.get("get_target").decode('ascii')

while first_pass is True or get_target == 'True':
    get_target = redis_server.get("get_target").decode('ascii')

    if get_target == 'True':
        first_pass = False
        for ii in range(0,len(target_positions)):
            print('sending to position %i', ii)
            redis_server.set('target_xyz', '%.3f %.3f %.3f'
                % (target_positions[ii,0], target_positions[ii,1],
                   target_positions[ii,2]))
            time.sleep(10)
        break
    else:
        print('waiting for control...'
        pass
