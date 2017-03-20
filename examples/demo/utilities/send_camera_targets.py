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
#def main():
# create a server for the vision system to connect to
redis_server = redis.StrictRedis(host='localhost')
redis_server.set("controller_name", "Adaptive")

# set true if only want to loop through targets once
single_pass = True

# in camera coordinate system
target_positions = np.array([
    # [0.109, -0.038, 0.588],
    [0.409, -0.038, 0.588],
    [-0.118, -0.039, 0.667],
    # [0.212, -0.069, 0.510],
    [0.152, -0.269, 0.510],
    [0.108, -0.134, 0.463],
    [0.099, -0.141, 0.463],
    [0.095, -0.135, 0.455],
    [-0.097, 0.138, 0.682],
    [0.154, 0.064, 0.612],
    [-0.351, -0.147, 0.824],
    [-0.359, -0.136, 0.824],
    [-0.020, -0.031, 0.463],
    [-0.191, -0.032, 0.541]])
# first pass allows script to wait for control system to load
# and wait for user to start
first_pass = True
get_target = redis_server.get("get_target").decode('ascii')
num_targets = 3
counter = 0
reach_t_limit = 20  # in seconds
while 1:
    try:
        print('waiting for control to initialize...')

        while first_pass is True or get_target == 'True':
            # wait until until user starts main loop to send targets
            get_target = redis_server.get("get_target").decode('ascii')

            if get_target == 'True':
                # set first pass to false so loop stops depending on get_target
                first_pass = False
                # set network running true to mimic vision system
                redis_server.set("network_running", "True")

                # loop through target positions
                for ii in range(0,num_targets):
                    # check get target to see if control side quit script
                    get_target = redis_server.get("get_target").decode('ascii')
                    if get_target == 'True':
                        print('sending to position %i' % ii)
                        redis_server.set('target_xyz', '%.3f %.3f %.3f'
                            % (target_positions[ii,0], target_positions[ii,1],
                               target_positions[ii,2]))
                        print('Count ', counter)
                        print('Waiting for arm to adapt...')
                        counter += 1
                        for sec in range(0,reach_t_limit):
                            get_target = redis_server.get("get_target").decode('ascii')
                            if get_target == 'True':
                                time.sleep(1)
                        print('Time limit up')
                    else:
                        print('User quit, exiting script...')
                        break

                if single_pass is True:
                    print('Exiting script')
                    break
            else:
                pass

    except Exception as e:
        print(traceback.format_exc())

    finally:
        print('Setting \'network running\' to False')
        redis_server.set("network_running", "False")
        print('Stopping arm')
        redis_server.set("stop_arm", "True")

    # get set up to loop again
    first_pass = True
    # wait a few seconds to allow all redis parameters to be reset
    # by other scripts
    time.sleep(2)
