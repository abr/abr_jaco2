"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np
import abr_jaco2
import abr_control
import time

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    regenerate_functions=False, use_cython=True,
    use_simplify=False, hand_attached=False)

friction = abr_jaco2.signals.friction(robot_config)
# instantiate the REACH controller for the jaco2 robot
kp = 25.0
kv = 5.0
loop_limit = 3000

ctrlr = abr_control.controllers.joint(robot_config, kp=kp, kv=kv)

ctrlr.control(np.zeros(robot_config.num_joints),
              np.zeros(robot_config.num_joints),
              np.zeros(robot_config.num_joints))

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)

target_pos = np.array([2.0, 2.75, 3.45, 1.0, .85, .5], dtype='float32')
target_vel = None
loop_count = 0

joint_angles = np.zeros((6, loop_limit))
torques_sent = np.zeros((6, loop_limit))
torques_read = np.zeros((6, loop_limit))
times = np.zeros(loop_limit)

# connect to the jaco
interface.connect()
interface.init_position_mode()

try:
    interface.apply_q(robot_config.home_position)
    interface.init_force_mode()
    start = time.time()
    while loop_count < loop_limit - 1:

        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0
        t_feedback = interface.get_torque_load()

        u = ctrlr.control(
            q=q, dq=dq,
            target_pos=target_pos, target_vel=target_vel)

        #print('no friction u: ', u)
        #u += friction.generate(dq=dq)
        #print('with friction u: ', u)
        interface.apply_u(np.array(u, dtype='float32'))

        times[loop_count] = time.time()-start

        # store variables
        joint_angles[:, loop_count] = np.copy(q)
        torques_read[:, loop_count] = np.copy(t_feedback['torque_load'])
        torques_sent[:, loop_count] = np.copy(u)

        loop_count += 1

except Exception as e:
    print(e)

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    interface.disconnect()

    if loop_count > 0:  # i.e. if it successfully ran
        error = np.sqrt(np.sum((torques_read - torques_sent)**2))
        np.savez_compressed('error', error=error)
        np.savez_compressed('kp', kp=kp)
        np.savez_compressed('kv', kv=kv)
        np.savez_compressed('target_pos', target_pos=target_pos)
        np.savez_compressed('joint_angles', joint_angles=joint_angles)
        np.savez_compressed('torques_sent', torques_sent=torques_sent)
        np.savez_compressed('torques_read', torques_read=torques_read)
        np.savez_compressed('times', times=times)

