"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np
import abr_jaco2
import abr_control

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    regenerate_functions=True, use_cython=True,
    use_simplify=False, hand_attached=False)
# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.joint(robot_config, kp=4.0, kv=2.0)

ctrlr.control(np.zeros(robot_config.num_joints),
              np.zeros(robot_config.num_joints),
              np.zeros(robot_config.num_joints))

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)

target_pos = np.array([2.0, 2.75, 3.45, 1.0, .5, .5], dtype='float32')
target_vel = None

# connect to the jaco
interface.connect()
interface.init_position_mode()
interface.apply_q(robot_config.home_position)

# set up arrays for tracking end-effector and target position
q_track = []
ctr = 0

try:
    interface.init_force_mode()
    while 1:
        ctr += 1
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0

        hand_xyz = robot_config.Tx('EE', q=q)

        u = ctrlr.control(q=q, dq=dq,
                          target_pos=target_pos, target_vel=target_vel)
        interface.apply_u(np.array(u, dtype='float32'))

        if ctr%1000 == 0:
            print('q: ', q)
        # set orientation of hand object to match EE
        """quaternion = robot_config.orientation('EE', q=q)
        angles = abr_control.utils.transformations.euler_from_quaternion(
            quaternion, axes='rxyz')
        interface.set_orientation('hand', angles)"""

        q_track.append(np.copy(q))

except Exception as e:
    print(e)

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    interface.disconnect()

    if ctr > 0:  # i.e. if it successfully ran
        import matplotlib.pyplot as plt
        # import seaborn

        q_track = np.array(q_track)
        plt.plot(q_track)
        plt.plot(np.ones(q_track.shape) *
                 ((target_pos + np.pi) % (np.pi * 2) - np.pi),
                 'r--')
        plt.tight_layout()
        plt.show()
