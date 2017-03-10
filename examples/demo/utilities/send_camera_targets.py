"""
Checks redis for 'get_target' confirmation, then sends
arm to three positions, waiting 10 seconds between points
in camera coordinates to mimic vision system sending targets
"""
# TODO fix quitting so can exit while looping through targets
import numpy as np
import redis
import time
import traceback

# create a server for the vision system to connect to
redis_server = redis.StrictRedis(host='localhost')
redis_server.set("controller_name", "Adaptive")

# in camera coordinate system
target_positions = np.array([[0.195,-0.070,0.510],
                            [0.028,0.094,0.753],
                            [-0.198,-0.076,0.592]])
# first pass allows script to wait for control system to load
# and wait for user to start
first_pass = True
get_target = redis_server.get("get_target").decode('ascii')

try:
    while first_pass is True or get_target == 'True':
        # wait until until user starts main loop to send targets
        get_target = redis_server.get("get_target").decode('ascii')

        if get_target == 'True':
            # set first pass to false so loop stops depending on get_target
            first_pass = False
            # set network running true to mimic vision system
            redis_server.set("network_running", "True")

            # loop through target positions
            for ii in range(0,len(target_positions)):
                get_target = redis_server.get("get_target").decode('ascii')
                    if get_target == 'True':
                        print('sending to position %i' % ii)
                        redis_server.set('target_xyz', '%.3f %.3f %.3f'
                            % (target_positions[ii,0], target_positions[ii,1],
                               target_positions[ii,2]))
                        time.sleep(10)
           # break
        else:
            print('waiting for control...')

except Exception as e:
    print(traceback.format_exc())

finally:
    print('Setting \'network running\' to False')
    redis_server.set("network_running", "False")
