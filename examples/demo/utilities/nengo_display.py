import nengo
import redis
import numpy as np
import struct
r = redis.StrictRedis(host='192.168.0.3')
def read_target_info(t):
    t_xyz = r.get('transformed')
    n_xyz = r.get('normalized')
    if t_xyz is None:
        t_xyz = np.zeros(3)
    else:
        t_xyz = np.array([float(v) for v in t_xyz.decode('ascii').split(',')])
    if n_xyz is None:
        n_xyz = np.zeros(3)
    else:
        n_xyz = np.array([float(v) for v in n_xyz.decode('ascii').split(',')])

    scaling = np.linalg.norm(t_xyz) / np.linalg.norm(n_xyz)

    if scaling > 1.0:
        read_target_info._nengo_html_ = '<center><h1>OUTSIDE</h1>%1.3f</center>' % scaling
    else:
        read_target_info._nengo_html_ = '<center><h1>inside</h1></center>'

    return np.hstack([t_xyz, n_xyz, [scaling]])

def read_controller(t):
    name = r.get('controller_name')
    if name is not None:
        name = name.decode('ascii')
    if name == "Non-compliant": #red
        read_controller._nengo_html_ = '<center><p style="font-size:100px; ' \
            'font-weight:bold; color:red; border:3px; border-style:solid; border-color:black; padding: .5em">%s</p></center>' % name
    elif name == "Adaptive": #green
        read_controller._nengo_html_ = '<center><p style="font-size:100px; ' \
            'font-weight:bold; color:green; border:3px; border-style:solid; border-color:black; padding: .5em">%s</p></center>' % name
    elif name == "Compliant": #black
        read_controller._nengo_html_ = '<center><p style="font-size:100px; ' \
            'font-weight:bold; color:black; border:3px; border-style:solid; border-color:black; padding: .5em">%s</p></center>' % name
def receive_spikes(t):
        msg = r.get('spikes')
        v = np.zeros(25)
        if len(msg) > 0:
            ii = struct.unpack('%dI' % (len(msg)/4), msg)
            v[[ii]] = 1000.0
        return v
def read_error(t):
    error = r.get('error')
    if error is None:
        error = 0.0
    else:
        error = float(error.decode('ascii'))

    read_error._nengo_html_ = '<h1>%1.3gm</h1>' % error
    return error
model = nengo.Network()
with model:
    target_info = nengo.Node(read_target_info, size_in=0)
    error = nengo.Node(read_error)

    name = nengo.Node(read_controller)
    neuron_spikes = nengo.Node(receive_spikes, size_in=0)
