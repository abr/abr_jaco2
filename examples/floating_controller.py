"""moves jaco to various joint positions and switches to torque
mode and float controller, records joint positions. Test to 
quantify gravity compensation improvements"""
import numpy as np
import os
import time
import abr_control
import abr_jaco2
# --- NAME TEST FOR SAVING ---

# ---------- INITIALIZATION ----------
# initialize our robot config for the ur5
robot_config = abr_jaco2.robot_config(
    regenerate_functions=True, use_cython=True,
    use_simplify=False, hand_attached=False)

ctrlr = abr_control.controllers.floating(robot_config)
ctrlr.control(np.zeros(6), np.zeros(6))

interface = abr_jaco2.interface(robot_config)

interface.connect()
interface.init_position_mode()

# ---------- MAIN BODY ----------
# Move to home position
interface.apply_q(robot_config.home_position)

try:
    # move to read position ii
    interface.apply_q(robot_config.home_position)
    t_feedback = interface.get_torque_load()
    torque_load = np.array(t_feedback['torque_load'], dtype="float32")    
    t_feedback = interface.get_torque_load()
        
    interface.init_force_mode()
    
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0

        u = ctrlr.control(q=q, dq=dq)

        interface.apply_u(np.array(u, dtype='float32'))      

except Exception as e:
    print(e)

finally:
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    interface.disconnect()
    print('Disconnected')