"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np
import time
import abr_control
import abr_jaco2

kp = 10.0
kv = 3.0
loop_limit = 15000

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    use_cython=True, hand_attached=True)

# NOTE: right now, in the osc when vmax = None, velocity is compensated
# for in joint space, with vmax set it's in task space

# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.osc(
    robot_config, kp=kp, kv=kv, vmax=1.0, null_control=False)
# create signal to compensate for friction
friction = abr_jaco2.signals.friction(robot_config)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(6), np.zeros(6), target_pos=np.zeros(3))
friction.generate(dq=np.zeros(6))

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)
# connect to the jaco
interface.connect()
# move to the home position
interface.apply_q(robot_config.home_position_start)

# set up arrays for tracking end-effector and target position
ee_track = []
u_track = []
# q_track = []
# dq_track = []
target_track = []

count = 0
target_index = 0
at_target_count = 0

joint_angles = np.zeros((6, loop_limit))
torques_sent = np.zeros((6, loop_limit))
torques_read = np.zeros((6, loop_limit))
frictions = np.zeros((6, loop_limit))
#C = np.zeros((6, loop_limit))
#g = np.zeros((6, loop_limit))
#training = np.zeros((6, loop_limit))
velocities = np.zeros((6, loop_limit))
times = np.zeros(loop_limit)
#x_tilde = np.zeros((3, loop_limit))

# list of targets to move to
targets = [[.4, -.2, .70],
           [-.467, -.22, .78],
           [.467, -.22, .78],
           [.467, .22, .78],
           [-.467, .22, .78]]
target_xyz = targets[0]
print('Moving to first target: ', target_xyz)

# switch to torque control mode
interface.init_force_mode()
u = np.zeros(robot_config.num_joints     )

try:
    loop_count = 0
    start = time.time()
    while loop_count<loop_limit-1:
        feedback = interface.get_feedback()
        q = np.array(feedback['q'])
        dq = np.array(feedback['dq'])
        t_feedback = interface.get_torque_load()
        
        u = ctrlr.control(q=q, dq=dq, target_pos=target_xyz)
        friction_generated = friction.generate(dq=dq)
        u += friction_generated
        interface.send_forces(np.array(u, dtype='float32'))

        hand_xyz = robot_config.Tx('EE', q=q)
        error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
        if error < .01:
            # if we're at the target, start count
            # down to moving to the next target
            at_target_count += 1
            if at_target_count >= 200:
                target_index += 1
                if target_index > len(targets):
                    break
                else:
                    target_xyz = targets[target_index]
                    print('Moving to next target: ', target_xyz)
                at_target_count = 0

        ee_track.append(hand_xyz)
        u_track.append(np.copy(u))

        target_track.append(target_xyz)
        count += 1
        if count % 100 == 0:
            print('error: ', error)
            #print('q: ', q)

        # store variables
        times[loop_count] = time.time()-start
        joint_angles[:, loop_count] = np.copy(q)
        torques_read[:, loop_count] = np.copy(t_feedback['torque_load'])
        torques_sent[:, loop_count] = np.copy(u)
        frictions[:, loop_count] = np.copy(friction_generated)
        velocities[:, loop_count] = np.copy(dq)
        #C[:, loop_count] = np.copy(ctrlr.C)
        #g[:, loop_count] = np.copy(ctrlr.g)
        #training[:, loop_count] = np.copy(ctrlr.training_signal)
        #x_tilde[:, loop_count] = np.copy(ctrlr.x_tilde)
        loop_count += 1

except Exception as e:
     print(e)
#
finally:
    # return back to home position
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position_end)
    # close the connection to the arm
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)
    u_track = np.array(u_track)

    # plot targets and trajectory of end-effectory in 3D
    #abr_control.utils.plotting.plot_trajectory(ee_track, target_track)

    if loop_count > 0:  # i.e. if it successfully ran
        np.savez_compressed('ee_track', ee_track=ee_track)
        np.savez_compressed('target_track', target_track=target_track)
        np.savez_compressed('u_track', u_track=u_track)
        np.savez_compressed('error', error=error)
        np.savez_compressed('kp', kp=kp)
        np.savez_compressed('kv', kv=kv)
        np.savez_compressed('target_pos', target_pos=target_xyz)
        np.savez_compressed('joint_angles', joint_angles=joint_angles)
        np.savez_compressed('torques_sent', torques_sent=torques_sent)
        np.savez_compressed('torques_read', torques_read=torques_read)
        np.savez_compressed('friction', friction = frictions)
        np.savez_compressed('velocity', velocity = velocities)
        np.savez_compressed('F_brk', F_brk=robot_config.F_brk)
        np.savez_compressed('times', times=times)
        #np.savez_compressed('C', C=C)
        #np.savez_compressed('g', g=g)
        #np.savez_compressed('training', training=training)
        #np.savez_compressed('x_tilde', x_tilde=x_tilde)


    """plt.figure()
    plt.plot(np.array(u_track))
    # plt.subplot(2, 1, 1)
    # plt.plot(np.array(q_track))
    # plt.subplot(2, 1, 2)
    # plt.plot(np.array(dq_track), '--')
    plt.legend(range(6))
    plt.show()
    sys.exit()"""
