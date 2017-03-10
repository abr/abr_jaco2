import nengo
import redis
import numpy as np

r = redis.StrictRedis(host='192.168.0.3')

def read_target_info(t):
    t_xyz = r.get('target_xyz_robot_coords')
    n_xyz = r.get('normalized')
    if t_xyz is None:
        t_xyz = np.zeros(3)
    else:
        t_xyz = np.array([float(v) for v in t_xyz.decode('ascii').split(',')])
    if n_xyz is None:
        n_xyz = np.zeros(3)
    else:
        n_xyz = np.array([float(v) for v in n_xyz.decode('ascii').split(',')])

    try:
        scaling = np.linalg.norm(t_xyz) / np.linalg.norm(n_xyz)
    except:
        scaling = 0

    if scaling > 1.0:
        read_target_info._nengo_html_ = '<center><h1>OUTSIDE</h1>%1.3f</center>' % scaling
    else:
        read_target_info._nengo_html_ = '<center><h1>inside</h1></center>'

    return np.hstack([t_xyz, n_xyz, [scaling]])

def read_controller(t):
    name = r.get('controller_name')
    if name is not None:
        name = name.decode('ascii')
    read_controller._nengo_html_ = '<center><h1>%s</h1></center>' % name

def read_xyz(t):
    xyz = r.get('xyz')
    if xyz is None:
        xyz = np.zeros(3)
    else:
        xyz = xyz.decode('ascii') 
        xyz = [float(val) for val in xyz[1:-1].split()]

    read_xyz._nengo_html_ = '<h1>%s</h1>' % ','.join(['%1.3f'%v for v in xyz])
    return xyz


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

    xyz = nengo.Node(read_xyz)
