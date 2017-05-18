"""
A basic script for connecting and moving the arm to 4 targets
in joint space using force control. The joint angles are recorded
and plotted against the target angles once the final target is
reached, and the arm has moved back to its default resting position.
"""
import sys
import numpy as np
import traceback

try:
    import abr_control.controllers.joint as Joint
except ImportError:
    print("abr_control is not installed, for the most recent controllers"
          + "please install the abr_control repo")
    from .skeleton_joint import Joint
import abr_jaco2

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)
# instantiate the REACH controller for the jaco2 robot
ctrlr = Joint(robot_config, kp=4, kv=2)

zeros = np.zeros(robot_config.NUM_JOINTS)
ctrlr.control(zeros, zeros, zeros)

# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

target_pos = np.array([2.0, 1.4, 1.8, 1.0, .5, .6], dtype='float32')
target_vel = None

# connect to the jaco
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

# set up arrays for tracking end-effector and target position
q_track = []
ctr = 0


try:
    interface.init_force_mode()
    while 1:
        feedback = interface.get_feedback()

        u = ctrlr.control(q=feedback['q'], dq=feedback['dq'],
                          target_pos=target_pos, target_vel=target_vel)
        interface.send_forces(np.array(u, dtype='float32'))

        q_track.append(np.copy(feedback['q']))

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

    q_track = np.array(q_track)

    import matplotlib.pyplot as plt

    plt.plot(q_track)
    plt.gca().set_color_cycle(None)
    plt.plot(np.ones(q_track.shape) *
             ((target_pos + np.pi) % (np.pi * 2) - np.pi), '--')
    plt.legend(range(6))
    plt.tight_layout()
    plt.show()
    sys.exit()
