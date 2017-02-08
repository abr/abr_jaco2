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

# instantiate the REACH controller for the jaco2 robot
kp = 30.0
kv = 15.0
loop_limit = 4000

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
        joint_angles[:, loop_count] = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0
        t_feedback = interface.get_torque_load()
        torques_read[:, loop_count] = np.array(t_feedback['torque_load'], dtype="float32")    

        torques_sent[:, loop_count] = ctrlr.control(q=joint_angles[:, loop_count], dq=dq,
                          target_pos=target_pos, target_vel=target_vel)
        interface.apply_u(np.array(torques_sent[:, loop_count], dtype='float32'))

        times[loop_count] = time.time()-start

        loop_count += 1

except Exception as e:
    print(e)

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    interface.disconnect()

    if loop_count > 0:  # i.e. if it successfully ran
        import matplotlib.pyplot as plt

        for ii in range(0,6):
            plt.figure()
            plt.subplot(211)
            plt.title('Joint %i Angles, Kp = %f Kv = %f' % (ii, kp, kv))
            plt.xlabel('time(sec)')
            plt.ylabel('joint angle (rad)')
            plt.plot((joint_angles[ii,:] + np.pi) % (np.pi * 2) - np.pi)
            plt.plot(((np.ones(joint_angles[ii,:].shape) * target_pos[ii, None])  + np.pi) % (np.pi * 2) - np.pi, '--')
            plt.legend(range(6))

            plt.subplot(212)
            plt.title('Joint %i Torques' % ii)
            plt.xlabel('time(sec)')
            plt.ylabel('torque (Nm)')
            plt.plot(torques_read[ii,:], label='Read')
            plt.plot(torques_sent[ii,:], '--', label='Sent')
            plt.legend()
            plt.tight_layout()

        plt.show()
