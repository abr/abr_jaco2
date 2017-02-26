import numpy as np

import abr_jaco2

rc = abr_jaco2.robot_config()
# T_inv = rc.T_inv('camera', q=np.zeros(6))
# xyz = np.array([0.0, 0.0, 0.0, 1])
# print('xyz in camera coords: ', np.dot(T_inv, xyz))
Tx = rc.Tx('camera', x=[0.198, 0.105, 0.737], q=np.zeros(6))

print('xyz in robot coords: ', Tx)
