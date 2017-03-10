import redis
import time

import abr_control

r = redis.StrictRedis(host='192.168.0.3')

# create our config, interface, and connect to VREP
rc = abr_control.arms.jaco2.config(
    use_cython=True, hand_attached=True)
interface = abr_control.interfaces.vrep(rc)
interface.connect()

print('Waiting for vrep_display to be activated')

while 1:
    q = r.get('q').decode('ascii').split()
    target_xyz = [float(val) for val in
                  r.get('target_xyz').decode('ascii').split()]
    if q is not None:
        q = [float(val) for val in q]
        interface.set_position(q)
        interface.set_xyz('target', target_xyz)
        # TODO: play with sleep time, see how low can be without burning resources
        time.sleep(.001)
    else:
        time.sleep(1)

interface.disconnect()
