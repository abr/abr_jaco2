""" Arm moves to a predetermined number of read positions
where the camera reads the tooltip position and an average
is taken. From here the offset is calculated wrt to the EE
and the arm moves the tooltip to the target position
"""

"""
Change null space gains in osc.py to...
nkp = self.kp * .01
nkv = nkp*0.3
"""
import numpy as np
import time
import os
import abr_control
import abr_jaco2

# ---------- INITIALIZATION ----------
# initialize our robot config for the ur5
robot_config = abr_jaco2.robot_config(
    regenerate_functions=True, use_cython=True)

# instantiate the REACH controller
ctrlr = abr_control.controllers.osc.controller(
    robot_config, kp=675, kv=450, vmax=0.7)

# run controller once to generate functions / take care of overhead
# outside of the main loop (so the torque mode isn't exited)
ctrlr.control(np.zeros(6), np.zeros(6),target_state=np.zeros(6), x=[.1, .1, .1])

# create our VREP interface
interface = abr_jaco2.interface(robot_config)
interface.connect()

# ----------PARAMETERS ----------
# offset from camera to robot origin
cam_offset = np.array([0.0, -0.29, 1.01])

calc_offset = False
# tooltip offset
manual_offset = np.array([-0.00069435, -0.00826757, -0.3623683])

get_target_position = False
manual_target_xyz = np.array([0.169, 0.330, 1.159])

# Values must be in range of -360 to 360 degrees
read_positions = np.array([
    [225, 110, 100, 310, 40, -20],
    [270, 110, 85, 85, 0, 0],
    [305, 105, 105, -35, 25, -90]], dtype="float32")

# saves calculated offset to average
offset_buf = np.zeros((3,len(read_positions)))

# home position tilted forward to avoid hitting camera mount used before home
# position so moving to home will just be tilting the arm back
prehome_pos = np.copy(robot_config.home_position)
prehome_pos[1] = prehome_pos[1] - 30

def get_offset(q, xyz_vision, cam_offset, ctr):
    # get joint positions
    print('q received in get_offset %i: ' % ctr, q)

    """ coordinates from camera point of view
    positive x-axis points to the right, the positive y-axis points down,
    and the positive z-axis points forward"""

    xyz_vision_2 = [float(i) for i in xyz_vision]
    object_start = ([
        xyz_vision_2[0] + cam_offset[0],
        xyz_vision_2[2] + cam_offset[1],
        -xyz_vision_2[1] + cam_offset[2]])

    print('object_start in robot frame %i: ' % ctr, object_start)

    # get the inverse transform matrix to calculate object end point
    # in the reference frame of the robot's last link
    T_inv = robot_config.T_inv('EE', q=q)
    offset = np.dot(T_inv, np.hstack([object_start, 1]))[:-1]
    print('offset from EE %i: ' % ctr, offset)
    return offset
# ==========================================================================

# ---------- MAIN BODY ----------
# Move to home position
interface.apply_q(prehome_pos)
interface.apply_q(robot_config.home_position)

if calc_offset is True:
    # Move to read positions
    for ii in range(0, len(read_positions)):
        # move to read position ii
        print('Moving to read position ', ii)
        interface.apply_q(read_positions[ii])

        # obtain tooltip position
        xyz_vision = cart_pos()
        print('xyz_vision received: ', xyz_vision)

        # get joint positions
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0

        # calculate offset
        offset_buf[ii] = get_offset(q, xyz_vision, cam_offset, ii)

    # calculate average offset
    offset = offset_buf.mean(axis=0)
    print('Average Offset: ', offset)

    # return to home position so we can switch to torque mode
    interface.apply_q(prehome_pos)
    interface.apply_q(robot_config.home_position)
else:
    offset = manual_offset

if get_target_position is True:
    # Get target position from vision
    target_xyz_temp = cart_pos()
    target_xyz = [float(i) for i in target_xyz_temp]
    target_xyz = ([
            target_xyz[0] + cam_offset[0],
            target_xyz[2] + cam_offset[1],
            -target_xyz[1] + cam_offset[2]])
else:
    target_xyz = manual_target_xyz

# pause before moving to target
time.sleep(0)

try:
    count = 0
    loop_count = 0
    print('target_xyz: ', target_xyz)
    start_loop = True

    while 1:
        loop_count += 1
        # get arm feedback
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0

        # generate a control signal
        u = ctrlr.control(q=q, dq=dq, x = offset,
                          target_state=np.hstack([target_xyz,np.zeros(3)]))

        # calculate tooltip endpoint in cartesian coordinates
        ee_xyz_control = robot_config.Tx('EE', q=q, x = offset)

        if start_loop is True:
            interface.init_force_mode()
            start_loop = False

        interface.apply_u(np.array(u, dtype='float32'))

        euclidean_err = np.sqrt(np.sum((target_xyz - ee_xyz_control)**2))
        xyz_err = target_xyz - ee_xyz_control
        # break once end-effector is within 5mm of the target

        if loop_count % 100 == 0:
            print('euclidean error: ', euclidean_err)
            print('xyz error: ', xyz_err)

        if euclidean_err < .01:
            count += 1
            if count > 200:
                print("At Target Position")
                break

finally:
    # stop and reset the VREP simulation
    interface.init_position_mode()
    # wait at  target for 3 seconds
    time.sleep(1)
    interface.apply_q(robot_config.home_position)
    interface.disconnect()
