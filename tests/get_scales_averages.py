"""float controller only accounts for gravity
You have timer minutes to move the arm around
in the space you want to work in at the desired /
expected speed.

After the timer completes the arm will move back 
to home position and prinout the scales and averages"""
import numpy as np
import abr_control
import abr_jaco2
import time
# --- NAME TEST FOR SAVING ---

# ---------- INITIALIZATION ----------
# initialize our robot config 
robot_config = abr_jaco2.robot_config(
    regenerate_functions=False, use_cython=True,
    hand_attached=False)

ctrlr = abr_control.controllers.floating(robot_config)
ctrlr.control(np.zeros(6), np.zeros(6))

interface = abr_jaco2.interface(robot_config)

interface.connect()
interface.init_position_mode()

timer = 30 #in seconds
qs = []
dqs = []
# ---------- MAIN BODY ----------
# Move to home position
interface.apply_q(robot_config.home_position_start)

try:
    # move to read position ii
    t_feedback = interface.get_torque_load()
    torque_load = np.array(t_feedback['torque_load'], dtype="float32")
    t_feedback = interface.get_torque_load()

    interface.init_force_mode()
    start = time.time()
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        q = np.array(feedback['q'])
        dq = np.array(feedback['dq'])

        u = ctrlr.control(q=q, dq=dq)
        interface.send_forces(np.array(u, dtype='float32'))
        qs.append(q)
        dqs.append(dq)

        if time.time() - start > timer:
            break;

except Exception as e:
    print(e)

finally:
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position_end)
    interface.disconnect()
    print('Disconnected')

    qs = np.array(qs)
    dqs = np.array(dqs)
    
    print('q means: ', qs.mean(axis=0))
    print('dq means: ', dqs.mean(axis=0))
    print('q scales: ', np.amax(qs,axis=0) - np.amin(qs,axis=0))
    print('dq scales: ', np.amax(dqs,axis=0) - np.amin(dqs,axis=0))