import numpy as np

import abr_control
import abr_jaco2

# initialize our robot config for the ur5
robot_config = abr_control.arms.ur5(
    use_cython=True)

# create our VREP interface
interface = abr_control.interfaces.vrep(
    robot_config, dt=.001)
interface.connect()

try:
    num_targets = 0
    count = 0
    back_to_start = False

    # get visual position of end point of object
    feedback = interface.get_feedback()
    object_start = np.array(interface.get_xyz('object_endpoint'))
    # get the inverse transform matrix to calculate object end point
    # in the reference frame of the robot's last link
    T_inv = robot_config.T_inv('EE', q=feedback['q'])
    offset = np.dot(T_inv, np.hstack([object_start, 1]))[:-1]

    target_xyz = object_start + np.array([.25, -.25, -.25])
    interface.set_xyz(name='target', xyz=target_xyz)

    while count < 200:
        #print('count: ', count)
        # get arm feedback from VREP
        feedback = interface.get_feedback()

        # generate a control signal
        #u = ctrlr.control(q=feedback['q'], dq=feedback['dq'], offset = offset,
        #                  target_x=target_xyz)

        # use visual feedback to get object endpoint position
        ee_xyz_visual = np.array(interface.get_xyz('object_endpoint'))
        ee_xyz_control = robot_config.Tx('EE', q=feedback['q'], x = offset)
        print('offset: ', offset)
        print('q: ', feedback['q'])
        print('ee_xyz_control: ', ee_xyz_control)
        # apply the control signal, step the sim forward
        #interface.apply_u(u)

        error = np.sqrt(np.sum((target_xyz - ee_xyz_control)**2))
        # break once end-effector is within 5mm of the target
        if error < .005:
            count += 1
            #break

        # track data
        #q_path.append(np.copy(feedback['q']))
        #dq_path.append(np.copy(feedback['dq']))
        #ee_path.append(np.copy(ee_xyz))
        #target_path.append(np.copy(target_xyz))

        if count % 50 == 0:
            print('error visual: %.4f' % (np.sqrt(np.sum((target_xyz - ee_xyz_visual)**2))))
            print('error control: %.4f' % (np.sqrt(np.sum((target_xyz - ee_xyz_control)**2))))
            print('ee diff (v-c): ', ((ee_xyz_visual-ee_xyz_control)))

finally:
    # stop and reset the VREP simulation
    interface.disconnect()
