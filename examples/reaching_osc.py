""" Example script of moving the arm to 3 targets using OSC """

import numpy as np
import traceback
import timeit

from abr_control.controllers import OSC, path_planners, signals
import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True)

# instantiate operation space controller
null_controllers = []
from abr_control.controllers import Damping
null_controllers.append(Damping(
    robot_config=robot_config, kv=1))

ctrlr_dof = [True, True, True, False, False, False]

ctrlr = OSC(robot_config, kp=14, kv=6, ki=0.002,
            null_controllers=null_controllers,
            ctrlr_dof=ctrlr_dof)
# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, zeros)
# offset to move control point from palm to fingers
robot_config.Tx('EE', q=zeros)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

# connect to the jaco
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.START_ANGLES)

target_xyz = np.array([[.56, -.09, .72],
                       [.12, .15, .75],
                       [.60, .26, .61],
                       [.38, .46, .81]])

# set up arrays for tracking end-effector and target position
error_track = []

# instantiate path planner and set parameters
path = path_planners.SecondOrderDMP(n_timesteps=500, error_scale=1e-6)

try:
    count = 0
    count_at_target = 0 #  must stay at target for 200 loop cycles for success
    target_index = 0
    run_time = 0
    times = []

    interface.connect()
    interface.init_position_mode()
    interface.send_target_angles(robot_config.START_ANGLES)

    feedback = interface.get_feedback()
    hand_xyz = robot_config.Tx('EE', q=feedback['q'])

    path.reset(position=hand_xyz, target_pos=target_xyz[target_index])
    generate_path = False

    # connect to the jaco
    interface.init_force_mode()

    while target_index < len(target_xyz):
        if generate_path:
            print('Generating next path')
            path.reset(position=hand_xyz, target_pos=target_xyz[target_index])
            generate_path = False
            print('Ready')

        start = timeit.default_timer()
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', q=feedback['q'])
        hand_vel = np.dot(robot_config.J('EE', feedback['q']),
                        feedback['dq'])[:3]

        error = np.sqrt(np.sum((hand_xyz - target_xyz[target_index])**2))
        target, target_vel = path._step(error)

        # generate the control signal
        u = ctrlr.generate(
            q=feedback['q'], dq=feedback['dq'],
            target=np.hstack((target, [0, 0, 0])),
            target_vel=np.hstack((target_vel, [0, 0, 0]))
            )

        interface.send_forces(np.array(u, dtype='float32'))

        # print out the error every so often
        if count % 100 == 0:
            print('error: ', error)

        # track data
        error_track.append(np.copy(error))

        # if within 5cm of target for 200 time steps move to next target
        if error < .02:
            count_at_target += 1
            if count_at_target >= 200:
                count_at_target = 0
                target_index += 1
                generate_path = True

        count+=1
        loop_time = timeit.default_timer() - start
        run_time += loop_time
        times.append(loop_time*1000)

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.send_target_angles(robot_config.START_ANGLES)
    interface.disconnect()

    import matplotlib
    matplotlib.use("TKAgg")
    import matplotlib.pyplot as plt
    fig = plt.figure()
    a1 = fig.add_subplot(211)
    a1.set_title("Trajectory Error")
    a1.plot(error_track)
    a1.set_ylabel("Distance to target [m]")
    a2 = fig.add_subplot(212)
    a2.plot(times, label=np.mean(times))
    a2.set_title('Loop Times')
    a2.legend()
    a2.set_ylabel('Time Steps [ms]')
    plt.tight_layout()
    plt.show()
