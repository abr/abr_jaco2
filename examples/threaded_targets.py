"""
Example of threading path planning with control for the real jaco2.
The jaco2 requires control commands every 200ms or so when in command mode.
To plan the next path the path planner must be thhreaded to allow for force
commands to be sent to maintain the current position before starting the next
path.

A mujoco sim implementation is also included for testing and can be run by
setting sim=True on line 29
"""
import traceback
import time
import numpy as np
import sys
import glfw

from threading import Thread
import matplotlib.pyplot as plt

from abr_control.controllers import OSC, Damping
from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
from abr_control.utils import transformations as transform
from abr_control.utils import colors

"""
"""
sim = True
dt = 0.001
# the error to be below (in meters) for target_error_count steps
# before moving on to the next target
error_thres = 0.02
target_error_count = 500 # number of steps to maintain sub error_thres error level
# how many steps passed the end of our path to allow the arm to reach the target
# before forcing it to move onto the next target
step_limit = 4000

if sim:
    kp = 30
    kv = 20
    ko = 180
else:
    kp = 23
    kv = 4.79
    ko = 78.2
# ctrlr_dof = [True, True, True, False, False, False] # x, y, z, a, b, g
# WARNING: the jaco2 is limitted in 6DOF control due to the non-right angle wrist joints
# it is recommended to constrain it to 4- 5 DOF control
ctrlr_dof = [True, True, True, True, False, False] # x, y, z, a, b, g

# euler axes order
axes = 'rxyz'

if sim:
    # run in mujoco
    from abr_control.arms.mujoco_config import MujocoConfig
    from abr_control.interfaces.mujoco import Mujoco
    robot_config = MujocoConfig("jaco2",)
    interface = Mujoco(robot_config, dt=dt)
else:
    # run on real arm
    import abr_jaco2
    robot_config = abr_jaco2.Config()
    interface = abr_jaco2.Interface(robot_config)

# define your targets
# n_targets = 10
# targets = []
# for tt in range(0, n_targets):
#     pos = np.random.uniform(0.4
targets = [
    {
        'pos': [0.5, 0.0, 0.5],
        'ori': [0, 0, 3.14],
        'start_v': 0,
        'target_v': 0,
        'max_v': 1,
    },
    {
        'pos': [0.5, 0.5, 0.6],
        'ori': [0, 0.4, 2.6],
        'start_v': 0,
        'target_v': 0,
        'max_v': 1,
    },
    {
        'pos': [0.0, 0.5, 0.4],
        'ori': [1, 0, 0],
        'start_v': 0,
        'target_v': 0,
        'max_v': 1,
    },
    {
        'pos': [-0.2, 0.4, 0.5],
        'ori': [3.2, 0, 0],
        'start_v': 0,
        'target_v': 0,
        'max_v': 1,
    }
]

try:
    Pprof = Linear()
    Vprof = Gaussian(dt=0.001, acceleration=1)
    path_planner = PathPlanner(pos_profile=Pprof, vel_profile=Vprof, verbose=True)

    # wrapper for path planner threading
    def gen_path(q, target):
        # get our start EE position and quat orientation
        start_pos = robot_config.Tx('EE', feedback['q'])
        start_quat = robot_config.quaternion('EE', feedback['q'])
        start_euler = transform.euler_from_quaternion(start_quat, axes)

        # generate the first path before turning on force mode
        path_planner.generate_path(
            start_position=start_pos,
            target_position=target['pos'],
            start_orientation=start_euler,
            target_orientation=target['ori'],
            max_velocity=target['max_v'],
            start_velocity=target['start_v'],
            target_velocity=target['target_v']
        )

    for target_count, target in enumerate(targets):
        print_cnt = 0 # for printing every 100 steps
        ii = -1 # for keeping track of progress along path
        step_limit_cnt = 0 # for keeping track of how many steps passed the path length have passed

        if target_count == 0:
            interface.connect()

            interface.send_target_angles(robot_config.START_ANGLES)
            feedback = interface.get_feedback()

            # OPTIONAL NULL SPACE CONTROLLER
            # reduce jerk in arm motion in the null space
            damping = Damping(robot_config, kv=4)

            # create opreational space controller
            ctrlr = OSC(robot_config, kp=kp, ko=ko, kv=kv, null_controllers=[damping],
                        vmax=None,
                        ctrlr_dof=ctrlr_dof)

            # generate the first path before starting force mode
            gen_path(q=feedback['q'], target=target)

            if not sim:
                # real arm needs to init force mode
                interface.init_force_mode()
            else:
                # for properly closing mujoco sim
                exit_sim = False

        # tracks if all conditions are met to continue to next target
        move_to_next_target = False
        path_gen_thread_started = False
        next_path_ready = False
        final_path_point = np.copy(path_planner.position_path[-1])

        # make sure we are at error thres and the next path is ready
        while not move_to_next_target:
            print_cnt += 1
            ii += 1

            # get arm feedback
            feedback = interface.get_feedback()

            # calculate error and track if below threshold
            hand_xyz = robot_config.Tx('EE', feedback['q'])#, x=approach_buffer)
            error = np.linalg.norm(hand_xyz - final_path_point[:3])

            # track how many consecutive steps we are below error thres
            if error < error_thres:
                at_error_count += 1
            else:
                at_error_count = 0

            if sim:
                # check for closing sim
                if interface.viewer.exit:
                    glfw.destroy_window(interface.viewer.window)
                    exit_sim = True
                    break

            filtered_target = path_planner.next()

            # at end of path, start counting towards our limit
            if ii > len(path_planner.position_path):

                # check if at the end of the target list
                if target_count < len(targets)-1:
                    # if not at last target, check if thread started to gen next path
                    if not path_gen_thread_started:
                        path_gen_thread_started = True
                        # === START THREAD TO GEN NEXT PATH ===
                        path_thread = Thread(
                            target=gen_path,
                            args=(
                                np.copy(feedback['q']),
                                target,
                            )
                        )
                        path_thread.start()
                    elif path_thread.is_alive():
                        next_path_ready = False
                    elif not path_thread.is_alive():
                        next_path_ready = True
                        path_thread.handled = True
                else:
                    # there is no next path, so set to True to exit loop
                    next_path_ready = True

                # at the end of the current seq path, track how many steps pass
                step_limit_cnt += 1

            if next_path_ready:
                # check within error thres for set number of steps
                if at_error_count > target_error_count:
                    print(f"{colors.green}AT ERROR COUNT CHECK: PASS{colors.endc}")
                    move_to_next_target = True
                elif step_limit_cnt > step_limit:
                    move_to_next_target = True
                    print(f"{colors.red}REACHED STEP LIMIT{colors.endc}")

            if print_cnt % 1000 == 0:
                hand_abg = transform.euler_from_matrix(
                    robot_config.R("EE", feedback["q"]), axes=axes
                )
                print(f"pos error: {error} meters")
                print('xyz: ', hand_xyz-final_path_point[:3])
                print('abg: ', np.asarray(list(hand_abg))-np.asarray(list(target['ori'])))

            u = ctrlr.generate(
                q=feedback['q'],
                dq=feedback['dq'],
                target=np.hstack((
                    filtered_target[:3],
                    filtered_target[6:9])),
                target_velocity=np.hstack((
                    filtered_target[3:6],
                    filtered_target[9:]))
                )

            if not sim:
                interface.send_forces(np.array(u, dtype='float32'))
            else:
                # interface.send_forces(np.hstack((u, u_grip)))
                u = np.hstack((u, np.zeros(robot_config.N_GRIPPER_JOINTS)))
                interface.send_forces(u)

                # visualization for debugging
                interface.set_mocap_xyz(
                    "target_orientation",
                    filtered_target[:3]
                )
                interface.set_mocap_orientation(
                    "target_orientation",
                    transform.quaternion_from_euler(
                        filtered_target[6],
                        filtered_target[7],
                        filtered_target[8],
                        axes=axes
                    )
                )

            if exit_sim:
                break

except Exception as e:
    print(e)
    print(traceback.format_exc())
finally:
    if not sim:
        interface.init_position_mode()
        interface.send_target_angles(robot_config.START_ANGLES)
    interface.disconnect()
