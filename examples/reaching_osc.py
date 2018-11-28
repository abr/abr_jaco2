""" Example script of moving the arm to 3 targets using OSC """

import numpy as np
import traceback

from abr_control.controllers import OSC, path_planners
import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)

# instantiate operation space controller
ctrlr = OSC(robot_config, kp=30, kv=6, ki=0.02, vmax=1,
            null_control=True)
# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, np.zeros(3))
# offset to move control point from palm to fingers
robot_config.Tx('EE', q=zeros, x=robot_config.OFFSET)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)
target_xyz = np.array([[.56, -.09, .95],
                       [.12, .15, .80],
                       [.80, .26, .61],
                       [.38, .46, .81]])

target_xyz = np.array([
              #[0.55699296, -0.12282506, 0.52730214],
              [0.54564553, 0.13975671, 0.52818036]
             ])
# instantiate path planner and set parameters
path = path_planners.SecondOrder(
    robot_config, n_timesteps=2000,
    w=1e4, zeta=2, threshold=0.08)
dt = 0.003

# connect to the jaco
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

# set up arrays for tracking end-effector and target position
ee_track = []
target_track = []

try:
    count = 0
    count_at_target = 0 #  must stay at target for 200 loop cycles for success
    target_index = 0

    feedback = interface.get_feedback()
    xyz = robot_config.Tx('EE', q=feedback['q'], x=robot_config.OFFSET)
    filtered_target = np.concatenate((xyz, np.array([0, 0, 0])), axis=0)

    interface.init_force_mode()
    while target_index < len(target_xyz):
        feedback = interface.get_feedback()
        xyz = robot_config.Tx('EE', q=feedback['q'], x=robot_config.OFFSET)

        filtered_target = path.step(
            state=filtered_target, target_pos=target_xyz[target_index])
        # generate the control signal
        u = ctrlr.generate(
            q=feedback['q'], dq=feedback['dq'],
            target_pos=filtered_target[:3],  # (x, y, z)
            #target_vel=filtered_target[3:],  # (dx, dy, dz)
            offset=robot_config.OFFSET)

        # additional gain term due to high stiction of jaco base joint
        # if u[0] > 0:
        #     u[0] *= 3.0
        # else:
        #     u[0] *= 2.0

        interface.send_forces(np.array(u, dtype='float32'))
        error = np.sqrt(np.sum((xyz - target_xyz[target_index])**2))

        # print out the error every so often
        if count % 100 == 0:
            print('error: ', error)

        # track data
        ee_track.append(np.copy(xyz))
        target_track.append(np.copy(target_xyz[target_index]))

        # if within 5cm of target for 200 time steps move to next target
        if error < .05:
            count_at_target += 1
            if count_at_target >= 200:
                count_at_target = 0
                target_index += 1

        count+=1

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib
        matplotlib.use("TKAgg")
        import matplotlib.pyplot as plt
        from abr_control.utils.plotting import plot_3D

        plt.figure()
        plt.plot(np.sqrt(np.sum((np.array(target_track)
                                 - np.array(ee_track))**2, axis=1)))
        plt.ylabel('Distance (m)')
        plt.xlabel('Time (ms)')
        plt.title('Distance to target')

        plot_3D(ee_track, target_track)
        plt.show()
