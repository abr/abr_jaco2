import numpy as np

import abr_control
import abr_jaco2

rc = abr_jaco2.robot_config(
    use_cython=True,
    hand_attached=True)
ctrlr = abr_control.controllers.osc(
    rc, kp=10, kv=3, vmax=1)
# T_inv = rc.T_inv('camera', q=np.zeros(6))
# xyz = np.array([0.0, 0.0, 0.0, 1])
# print('xyz in camera coords: ', np.dot(T_inv, xyz))
for ii in range(100):
  q = np.random.random(6)
  dq = np.random.random(6)
  target_xyz = np.random.random(3)

  # print('q: ', q)
  # Tx = rc.Tx('EE', q=q)
  # Tx_offset = rc.Tx('EE', x=[0, 0, 0.01], q=q)
  # print('Tx: \n', Tx)
  # print('Tx_offset: \n', Tx_offset)
  # print('difference: ', Tx - Tx_offset)
  #
  # J = rc.J('EE', q=q)
  # J_offset = rc.J('EE', x=[0, 0, 0.01], q=q)
  # print('J: \n', J)
  # print('J_offset: \n', J_offset)
  # print('difference: \n', J - J_offset)
  #
  # dJ = rc.dJ('EE', q=q, dq=dq)
  # dJ_offset = rc.dJ('EE', x=[0, 0, 0.01], q=q, dq=dq)
  # print('dJ: \n', dJ)
  # print('dJ_offset: \n', dJ_offset)
  # print('difference: \n', dJ - dJ_offset)

  u = ctrlr.control(
      q=q, dq=dq, target_pos=target_xyz)
  u_offset = ctrlr.control(
      q=q, dq=dq, offset=[0, 0, 0.00001], target_pos=target_xyz)

  print('u: ', u)
  print('u_offset: ', u_offset)
  print('difference: \n', u - u_offset)

  input()
